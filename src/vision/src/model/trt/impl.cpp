#include "booster_vision/model/trt/impl.h"
#include "booster_vision/model/trt/logging.h"

#include <fstream>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdexcept>


#if (NV_TENSORRT_MAJOR == 8) && (NV_TENSORRT_MINOR == 6)

#include "booster_vision/model/trt/cuda_utils.h"
#include "booster_vision/model/trt/model.h"
#include "booster_vision/model/trt/postprocess.h"
#include "booster_vision/model/trt/preprocess.h"
#include "booster_vision/model/trt/utils.h"

Logger gLogger;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSegSize = 32 * (kInputH / 4) * (kInputW / 4);

void serialize_det_engine(std::string& wts_name, std::string& engine_name, int& is_p, std::string& sub_type, float& gd,
                      float& gw, int& max_channels) {
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();
    IHostMemory* serialized_engine = nullptr;

    if (is_p == 6) {
        serialized_engine = buildEngineYolov8DetP6(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    } else if (is_p == 2) {
        serialized_engine = buildEngineYolov8DetP2(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    } else {
        serialized_engine = buildEngineYolov8Det(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    }

    assert(serialized_engine);
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cout << "could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    delete serialized_engine;
    delete config;
    delete builder;
}

void deserialize_det_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context) {
    std::cout << "loading det engine: " << engine_name << std::endl;
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

void prepare_det_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_buffer_host, float** decode_ptr_host, float** decode_ptr_device,
                    std::string cuda_post_process) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
    if (cuda_post_process == "c") {
        *output_buffer_host = new float[kBatchSize * kOutputSize];
    } else if (cuda_post_process == "g") {
        if (kBatchSize > 1) {
            std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
            exit(0);
        }
        // Allocate memory for decode_ptr_host and copy to device
        *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
        CUDA_CHECK(cudaMalloc((void**)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
    }
}

void infer_det(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
           float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process,
           float confidence_threshold, float nms_threshold) {
    // infer_det on the batch asynchronously, and DMA output back to host
    context.enqueue(batchsize, buffers, stream, nullptr);
    if (cuda_post_process == "c") {
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost,
                                   stream));
    } else if (cuda_post_process == "g") {
        CUDA_CHECK(
                cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float*)buffers[1], model_bboxes, confidence_threshold, decode_ptr_device, kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, nms_threshold, kMaxNumOutputBbox, stream);  //cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device,
                                   sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost,
                                   stream));
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void YoloV8DetectorTRT::Init(std::string model_path) {
  if (model_path.find(".engine") == std::string::npos) {
      throw std::runtime_error("incorrect model name: " + model_path);
  }

  deserialize_det_engine(model_path, &runtime, &engine, &context);

  CUDA_CHECK(cudaStreamCreate(&stream));
  cuda_preprocess_init(kMaxInputImageSize);
  auto out_dims = engine->getBindingDimensions(1);
  model_bboxes = out_dims.d[0];

 prepare_det_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host, &decode_ptr_host,
                &decode_ptr_device, cuda_post_process);
  std::cout << "det model initialization, done!"  << std::endl;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::Inference(const cv::Mat& img) {
  auto start = std::chrono::system_clock::now();
  // Preprocess
  std::vector<cv::Mat> img_batch = {img};
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

  std::vector<booster_vision::DetectionRes> ret;
  auto scale = std::min(kInputW / static_cast<float>(img.cols), kInputH / static_cast<float>(img.rows));
  auto offset_x = (kInputW - img.cols * scale) / 2;
  auto offset_y = (kInputH - img.rows * scale) / 2;

  cv::Mat s2d = (cv::Mat_<float>(2, 3) << scale, 0, offset_x, 0, scale, offset_y);
  cv::Mat d2s;
  cv::invertAffineTransform(s2d, d2s);
  
  for (auto res : res_batch[0]) {
    booster_vision::DetectionRes det_res;
    int x_min = std::max(0, static_cast<int>(res.bbox[0] * d2s.at<float>(0, 0) + d2s.at<float>(0, 2)));
    int y_min = std::max(0, static_cast<int>(res.bbox[1] * d2s.at<float>(1, 1) + d2s.at<float>(1, 2)));
    int x_max = std::min(img.cols - 1, static_cast<int>(res.bbox[2] * d2s.at<float>(0, 0) + d2s.at<float>(0, 2)));
    int y_max = std::min(img.rows - 1, static_cast<int>(res.bbox[3] * d2s.at<float>(1, 1) + d2s.at<float>(1, 2)));
    det_res.bbox = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    det_res.confidence = res.conf;
    det_res.class_id = res.class_id;
    ret.push_back(det_res);
  }
  auto end = std::chrono::system_clock::now();
  std::cout << "det inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms" << std::endl;
  return ret;
}

YoloV8DetectorTRT::~YoloV8DetectorTRT() {
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
}

#elif (NV_TENSORRT_MAJOR == 10) && (NV_TENSORRT_MINOR == 3)
Logger logger;

void YoloV8DetectorTRT::Init(std::string model_path) {
    if (model_path.find(".engine") == std::string::npos) {
        throw std::runtime_error("incorrect model name: " + model_path);
    }
    
    if (!LoadEngine()) {
        throw std::runtime_error("Failed to load engine from " + model_path);
    }

    input_size_ = model_input_dims_.d[0] * model_input_dims_.d[1] * model_input_dims_.d[2] * model_input_dims_.d[3];
    output_size_ = model_output_dims_.d[0] * model_output_dims_.d[1] * model_output_dims_.d[2];
    input_buff_ = (float*)malloc(input_size_ * sizeof(float));
    output_buff_ = (float*)malloc(output_size_ * sizeof(float));
    cudaMalloc(&input_mem_, input_size_ * sizeof(float));
    cudaMalloc(&output_mem_, output_size_ * sizeof(float));
    if (async_infer_)
    {
        cudaStreamCreate(&stream_);
    }
    else
    {
        bindings_.emplace_back(input_mem_);
        bindings_.emplace_back(output_mem_);
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_)
    {
        // return false;
        throw std::runtime_error("Failed to create execution context");
    }


    context_->setTensorAddress(engine_->getIOTensorName(0), input_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 1), output_mem_);
  
    std::cout << "det model initialization, done!"  << std::endl;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::Inference(const cv::Mat& img) {
    std::vector<float> factors;
    if (!PreProcess(img, factors))
    {
        return {};
    }

    // Memcpy from host input buffers to device input buffers
    MemcpyBuffers(input_mem_,input_buff_, input_size_ * sizeof(float),cudaMemcpyHostToDevice, async_infer_);

    bool status = false;
    if (async_infer_)
    {
        status = context_->enqueueV3(stream_);
    }
    else
    {
        status = context_->executeV2(bindings_.data());
    }
    
    if (!status)
    {
        return {};
    }

    // Memcpy from device output buffers to host output buffers
    MemcpyBuffers(output_buff_, output_mem_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost,async_infer_);

    if (async_infer_)
    {
        cudaStreamSynchronize(stream_);
    }

    img_width_ = img.cols;
    img_height_ = img.rows;
    std::vector<booster_vision::DetectionRes> outputs = PostProcess(factors);

    std::cout << "finish inference " << std::endl;
    return outputs;
}

YoloV8DetectorTRT::~YoloV8DetectorTRT() {
  cudaStreamDestroy(stream_);
  cudaFree(input_mem_);
  cudaFree(output_mem_);
  free(input_buff_);
  free(output_buff_);
}


bool YoloV8DetectorTRT::LoadEngine()
{
    std::ifstream input(model_path_, std::ios::binary);
    if (!input)
    {
        return false;
    }
    input.seekg(0, input.end);
    const size_t fsize = input.tellg();
    input.seekg(0, input.beg);
    std::vector<char> bytes(fsize);
    input.read(bytes.data(), fsize);

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(createInferRuntime(logger));
    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(bytes.data(), bytes.size()), InferDeleter());
    if (!engine_)
        return false;
    
    int nbio = engine_->getNbIOTensors();
    const char* inputname = engine_->getIOTensorName(0);
    const char* outputname = engine_->getIOTensorName(engine_->getNbIOTensors() - 1);
    Dims input_shape = engine_->getTensorShape(inputname);
    Dims output_shape = engine_->getTensorShape(outputname);
    model_input_dims_ = Dims4(input_shape.d[0], input_shape.d[1], input_shape.d[2], input_shape.d[3]);
    model_output_dims_ = Dims4(output_shape.d[0], output_shape.d[1], output_shape.d[2], output_shape.d[3]);
    std::cout << "model input dims: " << input_shape.d[0] << " " << input_shape.d[1] << " " << input_shape.d[2] << " " << input_shape.d[3] << std::endl;
    std::cout << "model output dims: " << output_shape.d[0] << " " << output_shape.d[1] << " " << output_shape.d[2] << std::endl;
   

    return true;
}

bool YoloV8DetectorTRT::PreProcess(const cv::Mat& img, std::vector<float>& factors)
{
    cv::Mat mat;
    int rh = img.rows;
    int rw = img.cols;
    int rc = img.channels();
    
    cv::cvtColor(img, mat, cv::COLOR_BGR2RGB);
    // mat = img;
    int maxImageLength = rw > rh ? rw : rh;
    if (squre_input_ && (model_input_dims_.d[2] == model_input_dims_.d[3]))
    {
        factors.emplace_back(maxImageLength / 640.0);
        factors.emplace_back(maxImageLength / 640.0);
    }
    else
    {
        factors.emplace_back(img.rows / 640.0);
        factors.emplace_back(img.cols / 640.0);
    }
    cv::Mat maxImage = cv::Mat::zeros(maxImageLength, maxImageLength, CV_8UC3);
    maxImage = maxImage * 255;
    cv::Rect roi(0, 0, rw, rh);
    mat.copyTo(cv::Mat(maxImage, roi));
    cv::Mat resizeImg;
    int length = 640;
    cv::resize(maxImage, resizeImg, cv::Size(length, length), 0.0f, 0.0f, cv::INTER_LINEAR);
    resizeImg.convertTo(resizeImg, CV_32FC3, 1 / 255.0);
    rh = resizeImg.rows;
    rw = resizeImg.cols;
    rc = resizeImg.channels();
    
    for (int i = 0; i < rc; ++i) {
        cv::extractChannel(resizeImg, cv::Mat(rh, rw, CV_32FC1, input_buff_ + i * rh * rw), i);
    }
    return true;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::PostProcess(std::vector<float> factors)
{
    const int outputSize = model_output_dims_.d[1];
    //float* output = static_cast<float*>(output_buff_);
    cv::Mat outputs(outputSize, 8400, CV_32F, output_buff_);

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    // Preprocessing output results
    const int class_num = outputSize - 4; // 4 for box[x,y,w,h]
    int rows = outputs.size[0];
    int dimensions = outputs.size[1];
    bool yolov8 = false;

    // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
    // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
    if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
    {
        yolov8 = true;
        rows = outputs.size[1];
        dimensions = outputs.size[0];

        outputs = outputs.reshape(1, dimensions);
        cv::transpose(outputs, outputs);
    }

    float* data = (float*)outputs.data;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        cv::Mat scores(1, class_num, CV_32FC1, classes_scores);
        // std::cout << "scores: " << scores << std::endl;
        cv::Point class_id;
        double maxClassScore;

        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

        if (maxClassScore > confidence_)
        {
            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            int left = int((x - 0.5 * w) * factors[0]);
            int top = int((y - 0.5 * h) * factors[1]);

            int width = int(w * factors[0]);
            int height = int(h * factors[1]);

            if (left < 0 || left > img_width_ - 1) continue;
            if (top < 0 || top > img_height_ - 1) continue;

            int right = std::min(img_width_ - 1, left + width);
            int bottom = std::min(img_height_ - 1, top + height);
            width = right - left;
            height = bottom - top;
            if (width < 3 || height < 3) continue;

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x);
        }

        data += dimensions;
    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, 0.25, 0.4, nms_result);

    std::vector<booster_vision::DetectionRes> detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        booster_vision::DetectionRes result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.bbox = boxes[idx];

        detections.push_back(result);
    }

    return detections;
}

void YoloV8DetectorTRT::MemcpyBuffers(void* dstPtr, void const* srcPtr, size_t byteSize, cudaMemcpyKind memcpyType, bool const async)
{
    if (async) {
        cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream_);
    } else {
        cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType);
    }
}

#endif