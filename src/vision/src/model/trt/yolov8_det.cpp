#include "booster_vision/model//trt/impl.h"
#include "booster_vision/model//trt/cuda_utils.h"
#include "booster_vision/model//trt/preprocess.h"
#include "booster_vision/model//trt/postprocess.h"
#include "booster_vision/model//trt/utils.h" //

#if (NV_TENSORRT_MAJOR == 8) && (NV_TENSORRT_MINOR == 6)

using namespace nvinfer1;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, int& is_p, std::string& img_dir,
                std::string& sub_type, std::string& cuda_post_process, float& gd, float& gw, int& max_channels) {
    if (argc < 4)
        return false;
    if (std::string(argv[1]) == "-s" && (argc == 5 || argc == 7)) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        auto sub_type = std::string(argv[4]);

        if (sub_type[0] == 'n') {
            gd = 0.33;
            gw = 0.25;
            max_channels = 1024;
        } else if (sub_type[0] == 's') {
            gd = 0.33;
            gw = 0.50;
            max_channels = 1024;
        } else if (sub_type[0] == 'm') {
            gd = 0.67;
            gw = 0.75;
            max_channels = 576;
        } else if (sub_type[0] == 'l') {
            gd = 1.0;
            gw = 1.0;
            max_channels = 512;
        } else if (sub_type[0] == 'x') {
            gd = 1.0;
            gw = 1.25;
            max_channels = 640;
        } else {
            return false;
        }
        if (sub_type.size() == 2 && sub_type[1] == '6') {
            is_p = 6;
        } else if (sub_type.size() == 2 && sub_type[1] == '2') {
            is_p = 2;
        }
    } else if (std::string(argv[1]) == "-d" && argc == 5) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
        cuda_post_process = std::string(argv[4]);
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
    int model_bboxes;
    int is_p = 0;
    float gd = 0.0f, gw = 0.0f;
    int max_channels = 0;

    if (!parse_args(argc, argv, wts_name, engine_name, is_p, img_dir, sub_type, cuda_post_process, gd, gw,
                    max_channels)) {
        std::cerr << "Arguments not right!" << std::endl;
        std::cerr << "./yolov8 -s [.wts] [.engine] [n/s/m/l/x/n2/s2/m2/l2/x2/n6/s6/m6/l6/x6]  // serialize model to "
                     "plan file"
                  << std::endl;
        std::cerr << "./yolov8 -d [.engine] ../samples  [c/g]// deserialize plan file and run inference" << std::endl;
        return -1;
    }

    // Create a model using the API directly and serialize it to a file
    if (!wts_name.empty()) {
        serialize_det_engine(wts_name, engine_name, is_p, sub_type, gd, gw, max_channels);
        return 0;
    }

    // Deserialize the engine from file
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    deserialize_det_engine(engine_name, &runtime, &engine, &context);
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    cuda_preprocess_init(kMaxInputImageSize);
    auto out_dims = engine->getBindingDimensions(1);
    model_bboxes = out_dims.d[0];
    // Prepare cpu and gpu buffers
    float* device_buffers[2];
    float* output_buffer_host = nullptr;
    float* decode_ptr_host = nullptr;
    float* decode_ptr_device = nullptr;

    // Read images from directory
    std::vector<std::string> file_names;
    if (read_files_in_dir(img_dir.c_str(), file_names) < 0) {
        std::cerr << "read_files_in_dir failed." << std::endl;
        return -1;
    }

    prepare_det_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host, &decode_ptr_host,
                   &decode_ptr_device, cuda_post_process);

    // batch predict
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
        infer_det(*context, stream, (void**)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host,
              decode_ptr_device, model_bboxes, cuda_post_process);
        std::vector<std::vector<Detection>> res_batch;
        if (cuda_post_process == "c") {
            // NMS
            batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
        } else if (cuda_post_process == "g") {
            //Process gpu decode and nms results
            batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
        }
        // Draw bounding boxes
        draw_bbox(img_batch, res_batch);
        // Save images
        for (size_t j = 0; j < img_batch.size(); j++) {
            cv::imwrite("_" + img_name_batch[j], img_batch[j]);
        }
    }

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    CUDA_CHECK(cudaFree(decode_ptr_device));
    delete[] decode_ptr_host;
    delete[] output_buffer_host;
    cuda_preprocess_destroy();
    // Destroy the engine
    delete context;
    delete engine;
    delete runtime;

    // Print histogram of the output distribution
    //std::cout << "\nOutput:\n\n";
    //for (unsigned int i = 0; i < kOutputSize; i++)
    //{
    //    std::cout << prob[i] << ", ";
    //    if (i % 10 == 0) std::cout << std::endl;
    //}
    //std::cout << std::endl;

    return 0;
}
#elif (NV_TENSORRT_MAJOR == 10) && (NV_TENSORRT_MINOR == 3)
int main(int argc, char** argv) {
	std::cout << "hello world!" << std::endl;
}

#endif
