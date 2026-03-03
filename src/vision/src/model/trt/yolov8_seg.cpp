
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "booster_vision/model/trt/cuda_utils.h"
#include "booster_vision/model/trt/logging.h"
#include "booster_vision/model/trt/model.h"
#include "booster_vision/model/trt/postprocess.h"
#include "booster_vision/model/trt/preprocess.h"
#include "booster_vision/model/trt/utils.h"
#include "booster_vision/model/trt/impl.h"

#if (NV_TENSORRT_MAJOR == 8) && (NV_TENSORRT_MINOR == 6)
Logger gLogger;
using namespace nvinfer1;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSegSize = 32 * (kInputH / 4) * (kInputW / 4);

extern std::vector<cv::Mat> process_mask(const float* proto, int proto_size, std::vector<Detection>& dets);

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, std::string& img_dir,
                std::string& sub_type, std::string& cuda_post_process, std::string& labels_filename, float& gd,
                float& gw, int& max_channels) {
    if (argc < 4)
        return false;
    if (std::string(argv[1]) == "-s" && argc == 5) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        sub_type = std::string(argv[4]);
        if (sub_type == "n") {
            gd = 0.33;
            gw = 0.25;
            max_channels = 1024;
        } else if (sub_type == "s") {
            gd = 0.33;
            gw = 0.50;
            max_channels = 1024;
        } else if (sub_type == "m") {
            gd = 0.67;
            gw = 0.75;
            max_channels = 576;
        } else if (sub_type == "l") {
            gd = 1.0;
            gw = 1.0;
            max_channels = 512;
        } else if (sub_type == "x") {
            gd = 1.0;
            gw = 1.25;
            max_channels = 640;
        } else {
            return false;
        }
    } else if (std::string(argv[1]) == "-d" && argc == 6) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
        cuda_post_process = std::string(argv[4]);
        labels_filename = std::string(argv[5]);
    } else {
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    cudaSetDevice(kGpuId);
    std::string wts_name = "";
    std::string engine_name = "";
    std::string img_dir;
    std::string sub_type = "";
    std::string cuda_post_process = "";
    std::string labels_filename = "../coco.txt";
    int model_bboxes;
    float gd = 0.0f, gw = 0.0f;
    int max_channels = 0;

    if (!parse_args(argc, argv, wts_name, engine_name, img_dir, sub_type, cuda_post_process, labels_filename, gd, gw,
                    max_channels)) {
        std::cerr << "Arguments not right!" << std::endl;
        std::cerr << "./yolov8 -s [.wts] [.engine] [n/s/m/l/x]  // serialize model to plan file" << std::endl;
        std::cerr << "./yolov8 -d [.engine] ../samples  [c/g] coco_file// deserialize plan file and run inference"
                  << std::endl;
        return -1;
    }

    // Create a model using the API directly and serialize it to a file
    if (!wts_name.empty()) {
        serialize_seg_engine(wts_name, engine_name, sub_type, gd, gw, max_channels);
        return 0;
    }

    // Deserialize the engine from file
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    deserialize_seg_engine(engine_name, &runtime, &engine, &context);
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    cuda_preprocess_init(kMaxInputImageSize);
    auto out_dims = engine->getBindingDimensions(1);
    model_bboxes = out_dims.d[0];
    // Prepare cpu and gpu buffers
    float* device_buffers[3];
    float* output_buffer_host = nullptr;
    float* output_seg_buffer_host = nullptr;
    float* decode_ptr_host = nullptr;
    float* decode_ptr_device = nullptr;

    // Read images from directory
    std::vector<std::string> file_names;
    if (read_files_in_dir(img_dir.c_str(), file_names) < 0) {
        std::cerr << "read_files_in_dir failed." << std::endl;
        return -1;
    }

    std::unordered_map<int, std::string> labels_map;
    read_labels(labels_filename, labels_map);
    assert(kNumClass == labels_map.size());

    prepare_seg_buffer(engine, &device_buffers[0], &device_buffers[1], &device_buffers[2], &output_buffer_host,
                   &output_seg_buffer_host, &decode_ptr_host, &decode_ptr_device, cuda_post_process);

    // // batch predict
    for (size_t i = 0; i < file_names.size(); i += kBatchSize) {
        // Get a batch of images
        std::vector<cv::Mat> img_batch;
        std::vector<std::string> img_name_batch;
        for (size_t j = i; j < i + kBatchSize && j < file_names.size(); j++) {
            cv::Mat img = cv::imread(img_dir + "/" + file_names[j]);
            img_batch.push_back(img);
            img_name_batch.push_back(file_names[j]);
        }
        // Preprocess
        cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);
        // Run inference
        infer_seg(*context, stream, (void**)device_buffers, output_buffer_host, output_seg_buffer_host, kBatchSize,
              decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);
        std::vector<std::vector<Detection>> res_batch;
        if (cuda_post_process == "c") {
            // NMS
            batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
            for (size_t b = 0; b < img_batch.size(); b++) {
                auto& res = res_batch[b];
                cv::Mat img = img_batch[b];
                auto masks = process_mask(&output_seg_buffer_host[b * kOutputSegSize], kOutputSegSize, res);
                draw_mask_bbox(img, res, masks, labels_map);
                cv::imwrite("_" + img_name_batch[b], img);
            }
        } else if (cuda_post_process == "g") {
            // Process gpu decode and nms results
            // batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
            // todo seg in gpu
            std::cerr << "seg_postprocess is not support in gpu right now" << std::endl;
        }
    }

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    CUDA_CHECK(cudaFree(device_buffers[2]));
    CUDA_CHECK(cudaFree(decode_ptr_device));
    delete[] decode_ptr_host;
    delete[] output_buffer_host;
    delete[] output_seg_buffer_host;
    cuda_preprocess_destroy();
    // Destroy the engine
    delete context;
    delete engine;
    delete runtime;

    // Print histogram of the output distribution
    // std::cout << "\nOutput:\n\n";
    // for (unsigned int i = 0; i < kOutputSize; i++)
    //{
    //    std::cout << prob[i] << ", ";
    //    if (i % 10 == 0) std::cout << std::endl;
    //}
    // std::cout << std::endl;

    return 0;
}

#elif (NV_TENSORRT_MAJOR == 10) && (NV_TENSORRT_MINOR == 3)
int main(int argc, char** argv) {
	std::cout << "hello world!" << std::endl;
}

#endif