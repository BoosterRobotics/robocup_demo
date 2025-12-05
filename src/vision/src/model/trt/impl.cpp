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
        decode_ptr_device, model_bboxes, cuda_post_process, confidence_, nms_threshold_);
  std::vector<std::vector<Detection>> res_batch;
  if (cuda_post_process == "c") {
      // NMS
      batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, confidence_, nms_threshold_);
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


static cv::Rect get_downscale_rect(float bbox[4], float scale) {

    float left = bbox[0];
    float top = bbox[1];
    float right = bbox[0] + bbox[2];
    float bottom = bbox[1] + bbox[3];

    left = left < 0 ? 0 : left;
    top = top < 0 ? 0 : top;
    right = right > kInputW ? kInputW : right;
    bottom = bottom > kInputH ? kInputH : bottom;

    left /= scale;
    top /= scale;
    right /= scale;
    bottom /= scale;
    return cv::Rect(int(left), int(top), int(right - left), int(bottom - top));
}

std::vector<cv::Mat> process_mask(const float* proto, int proto_size, std::vector<Detection>& dets) {
    std::vector<cv::Mat> masks;
    for (size_t i = 0; i < dets.size(); i++) {

        cv::Mat mask_mat = cv::Mat::zeros(kInputH / 4, kInputW / 4, CV_32FC1);
        auto r = get_downscale_rect(dets[i].bbox, 4);

        for (int x = r.x; x < r.x + r.width; x++) {
            for (int y = r.y; y < r.y + r.height; y++) {
                float e = 0.0f;
                for (int j = 0; j < 32; j++) {
                    e += dets[i].mask[j] * proto[j * proto_size / 32 + y * mask_mat.cols + x];
                }
                e = 1.0f / (1.0f + expf(-e));
                mask_mat.at<float>(y, x) = e;
            }
        }
        cv::resize(mask_mat, mask_mat, cv::Size(kInputW, kInputH));
        masks.push_back(mask_mat);
    }
    return masks;
}

void serialize_seg_engine(std::string& wts_name, std::string& engine_name, std::string& sub_type, float& gd, float& gw,
                      int& max_channels) {
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();
    IHostMemory* serialized_engine = nullptr;

    serialized_engine = buildEngineYolov8Seg(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);

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

void deserialize_seg_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context) {
    std::cout << "loading seg engine: " << engine_name << std::endl;
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

void prepare_seg_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_seg_buffer_device, float** output_buffer_host, float** output_seg_buffer_host,
                    float** decode_ptr_host, float** decode_ptr_device, std::string cuda_post_process) {
    assert(engine->getNbBindings() == 3);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    const int outputIndex_seg = engine->getBindingIndex("proto");

    assert(inputIndex == 0);
    assert(outputIndex == 1);
    assert(outputIndex_seg == 2);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_seg_buffer_device, kBatchSize * kOutputSegSize * sizeof(float)));

    if (cuda_post_process == "c") {
        *output_buffer_host = new float[kBatchSize * kOutputSize];
        *output_seg_buffer_host = new float[kBatchSize * kOutputSegSize];
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

void infer_seg(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, float* output_seg,
           int batchsize, float* decode_ptr_host, float* decode_ptr_device, int model_bboxes,
           std::string cuda_post_process) {
    // infer_seg on the batch asynchronously, and DMA output back to host
    context.enqueue(batchsize, buffers, stream, nullptr);
    if (cuda_post_process == "c") {
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost,
                                   stream));
        CUDA_CHECK(cudaMemcpyAsync(output_seg, buffers[2], batchsize * kOutputSegSize * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream));

    } else if (cuda_post_process == "g") {
        CUDA_CHECK(
                cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float*)buffers[1], model_bboxes, kConfThresh, decode_ptr_device, kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, kNmsThresh, kMaxNumOutputBbox, stream);  //cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device,
                                   sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost,
                                   stream));
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void YoloV8SegmentorTRT::Init(std::string model_path) {
  if (model_path.find(".engine") == std::string::npos) {
      throw std::runtime_error("incorrect model name: " + model_path);
  }

  deserialize_seg_engine(model_path, &runtime, &engine, &context);

  CUDA_CHECK(cudaStreamCreate(&stream));
  cuda_preprocess_init(kMaxInputImageSize);
  auto out_dims = engine->getBindingDimensions(1);
  model_bboxes = out_dims.d[0];

  prepare_seg_buffer(engine, &device_buffers[0], &device_buffers[1], &device_buffers[2], &output_buffer_host,
                &output_seg_buffer_host, &decode_ptr_host, &decode_ptr_device, cuda_post_process);
  std::cout << "seg model initialization, done!"  << std::endl;
}

YoloV8SegmentorTRT::~YoloV8SegmentorTRT() {
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
}

std::vector<booster_vision::SegmentationRes> YoloV8SegmentorTRT::Inference(const cv::Mat& img) {
  auto start = std::chrono::system_clock::now();
  // Preprocess
  std::vector<cv::Mat> img_batch = {img};
  cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);
  // Run inference
  infer_seg(*context, stream, (void**)device_buffers, output_buffer_host, output_seg_buffer_host, kBatchSize,
        decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);
  std::vector<std::vector<Detection>> res_batch;
  std::vector<cv::Mat> masks;
  if (cuda_post_process == "c") {
      // NMS
      batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, confidence_, nms_threshold_);
      for (size_t b = 0; b < img_batch.size(); b++) {
          auto& res = res_batch[b];
          cv::Mat img = img_batch[b];
          // compute mask from proto; mask size: (640, 640)
          masks = process_mask(&output_seg_buffer_host[b * kOutputSegSize], kOutputSegSize, res);
          // resize mask to original image size
          for (auto& mask : masks) {
              mask = scale_mask(mask, img) > 0.5;
            //   std::cout << "u8 dtype: " << (mask.type() == CV_8UC1) << std::endl;
            //   double min_val, max_val;
            //   cv::minMaxLoc(mask, &min_val, &max_val);
            //   std::cout << "mask min_val: " << min_val << ", max_val: " << max_val << std::endl;
          }
        //   draw_mask_bbox(img, res, masks, labels_map);
        //   cv::imwrite("_" + img_name_batch[b], img);
      }
  } else if (cuda_post_process == "g") {
      // Process gpu decode and nms results
      // batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
      // todo seg in gpu
      std::cerr << "seg_postprocess is not support in gpu right now" << std::endl;
  }

  std::vector<booster_vision::SegmentationRes> ret;
  auto scale = std::min(kInputW / static_cast<float>(img.cols), kInputH / static_cast<float>(img.rows));
  auto offset_x = (kInputW - img.cols * scale) / 2;
  auto offset_y = (kInputH - img.rows * scale) / 2;

  cv::Mat s2d = (cv::Mat_<float>(2, 3) << scale, 0, offset_x, 0, scale, offset_y);
  cv::Mat d2s;
  cv::invertAffineTransform(s2d, d2s);
  
  for (size_t i = 0; i < res_batch[0].size(); i++) {
    auto res = res_batch[0][i];
    booster_vision::SegmentationRes seg_res;
    int x_min = std::max(0, static_cast<int>(res.bbox[0] * d2s.at<float>(0, 0) + d2s.at<float>(0, 2)));
    int y_min = std::max(0, static_cast<int>(res.bbox[1] * d2s.at<float>(1, 1) + d2s.at<float>(1, 2)));
    int x_max = std::min(img.cols - 1, static_cast<int>(res.bbox[2] * d2s.at<float>(0, 0) + d2s.at<float>(0, 2)));
    int y_max = std::min(img.rows - 1, static_cast<int>(res.bbox[3] * d2s.at<float>(1, 1) + d2s.at<float>(1, 2)));
    seg_res.bbox = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    seg_res.confidence = res.conf;
    seg_res.class_id = res.class_id;

    cv::Mat mask_area = cv::Mat::zeros(img.size(), CV_8UC1);
    mask_area(seg_res.bbox).setTo(cv::Scalar(255));
    cv::bitwise_and(masks[i], mask_area, seg_res.mask);

    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(seg_res.mask, seg_res.contour, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
    ret.push_back(seg_res);
  }
  auto end = std::chrono::system_clock::now();
  std::cout << "seg inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms" << std::endl;
  return ret;
}

#elif (NV_TENSORRT_MAJOR == 10) && (NV_TENSORRT_MINOR == 3)

namespace {

Logger logger;

void MemcpyBuffers(void* dstPtr, void const* srcPtr, size_t byteSize, cudaMemcpyKind memcpyType, bool const async, cudaStream_t& stream) {
    if (async) {
        cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream);
    } else {
        cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType);
    }
}

bool PreProcess(const cv::Mat& img, const long int* model_input_shape, float* input_buff, std::vector<float>& factors) {
    cv::Mat mat;
    int rh = img.rows;
    int rw = img.cols;
    int rc = img.channels();
    
    cv::cvtColor(img, mat, cv::COLOR_BGR2RGB);
    // mat = img;
    int max_img_len = rw > rh ? rw : rh;
    const int model_input_width = model_input_shape[0];
    const int model_input_height = model_input_shape[1];
    assert(model_input_width == 640 && "model_input_width must be 640");
    if (model_input_width == model_input_height) {
        factors.emplace_back(max_img_len / model_input_width);
        factors.emplace_back(max_img_len / model_input_width);
    } else {
        factors.emplace_back(img.rows / model_input_width);
        factors.emplace_back(img.cols / model_input_width);
    }
    cv::Mat max_img = cv::Mat::zeros(max_img_len, max_img_len, CV_8UC3);
    max_img = max_img * 255;
    cv::Rect roi(0, 0, rw, rh);
    mat.copyTo(cv::Mat(max_img, roi));
    cv::Mat resized_img;
    cv::resize(max_img, resized_img, cv::Size(model_input_width, model_input_width), 0.0f, 0.0f, cv::INTER_LINEAR);
    resized_img.convertTo(resized_img, CV_32FC3, 1 / 255.0);
    rh = resized_img.rows;
    rw = resized_img.cols;
    rc = resized_img.channels();
    
    for (int i = 0; i < rc; ++i) {
        cv::extractChannel(resized_img, cv::Mat(rh, rw, CV_32FC1, input_buff + i * rh * rw), i);
    }
    return true;
}

cv::Rect ProcessBoundingBox(const float* data, const std::vector<float>& factors, 
                           int img_width, int img_height) {
    float x = data[0];
    float y = data[1];
    float w = data[2];
    float h = data[3];

    int left = int((x - 0.5f * w) * factors[0]);
    int top = int((y - 0.5f * h) * factors[1]);
    int width = int(w * factors[0]);
    int height = int(h * factors[1]);

    // Boundary checks
    if (left < 0 || left > img_width - 1 || top < 0 || top > img_height - 1) {
        return cv::Rect(0, 0, 0, 0); // Invalid rect
    }

    // Adjust dimensions to fit within image
    int right = std::min(img_width - 1, left + width);
    int bottom = std::min(img_height - 1, top + height);
    width = right - left;
    height = bottom - top;
    
    if (width < 3 || height < 3) {
        return cv::Rect(0, 0, 0, 0); // Too small
    }
    
    return cv::Rect(left, top, width, height);
}
}

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
    if (!PreProcess(img, model_input_dims_.d + 2, input_buff_, factors))
    {
        return {};
    }

    // Memcpy from host input buffers to device input buffers
    MemcpyBuffers(input_mem_,input_buff_, input_size_ * sizeof(float),cudaMemcpyHostToDevice, async_infer_, stream_);

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
    MemcpyBuffers(output_buff_, output_mem_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost,async_infer_, stream_);

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

    if (dimensions > rows)
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
        double max_class_score;

        minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

        if (max_class_score > confidence_)
        {
            auto bbox = ProcessBoundingBox(data, factors, img_width_, img_height_);
            if (bbox.width <= 0 || bbox.height <= 0) continue;

            boxes.push_back(bbox);
            confidences.push_back(max_class_score);
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


bool YoloV8SegmentorTRT::LoadEngine() {
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
    
    int nb_io = engine_->getNbIOTensors();
    const char* input_name = engine_->getIOTensorName(0);
    const char* output1_name = engine_->getIOTensorName(nb_io - 2);
    const char* output2_name = engine_->getIOTensorName(nb_io - 1);
    Dims input_shape = engine_->getTensorShape(input_name);
    Dims output1_shape = engine_->getTensorShape(output1_name);
    Dims output2_shape = engine_->getTensorShape(output2_name);
    model_input_dims_ = Dims4(input_shape.d[0], input_shape.d[1], input_shape.d[2], input_shape.d[3]);
    model_output1_dims_ = Dims4(output1_shape.d[0], output1_shape.d[1], output1_shape.d[2], output1_shape.d[3]);
    model_output2_dims_ = Dims4(output2_shape.d[0], output2_shape.d[1], output2_shape.d[2], output2_shape.d[3]);
    std::cout << "model input dims: " << input_shape.d[0] << " " << input_shape.d[1] << " " << input_shape.d[2] << " " << input_shape.d[3] << std::endl;
    std::cout << "model detection output dims: " << output1_shape.d[0] << " " << output1_shape.d[1] << " " << output1_shape.d[2] << std::endl;
    std::cout << "model mask output dims: " << output2_shape.d[0] << " " << output2_shape.d[1] << " " << output2_shape.d[2] << std::endl;

    return true;
}

void YoloV8SegmentorTRT::Init(std::string model_path) {
    if (model_path.find(".engine") == std::string::npos) {
        throw std::runtime_error("incorrect model name: " + model_path);
    }
    
    if (!LoadEngine()) {
        throw std::runtime_error("Failed to load engine from " + model_path);
    }

    input_size_ = model_input_dims_.d[0] * model_input_dims_.d[1] * model_input_dims_.d[2] * model_input_dims_.d[3];
    det_output_size_ = model_output1_dims_.d[0] * model_output1_dims_.d[1] * model_output1_dims_.d[2];
    mask_output_size_ = model_output2_dims_.d[0] * model_output2_dims_.d[1] * model_output2_dims_.d[2] * model_output2_dims_.d[3];
    input_buff_ = (float*)malloc(input_size_ * sizeof(float));
    det_output_buff_ = (float*)malloc(det_output_size_ * sizeof(float));
    mask_output_buff_ = (float*)malloc(mask_output_size_ * sizeof(float));

    cudaMalloc(&input_mem_, input_size_ * sizeof(float));
    cudaMalloc(&det_output_mem_, det_output_size_ * sizeof(float));
    cudaMalloc(&mask_output_mem_, mask_output_size_ * sizeof(float));
    if (async_infer_)
    {
        cudaStreamCreate(&stream_);
    }
    else
    {
        bindings_.emplace_back(input_mem_);
        bindings_.emplace_back(det_output_mem_);
        bindings_.emplace_back(mask_output_mem_);
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_)
    {
        // return false;
        throw std::runtime_error("Failed to create execution context");
    }


    context_->setTensorAddress(engine_->getIOTensorName(0), input_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 2), det_output_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 1), mask_output_mem_);
  
    std::cout << "seg model initialization, done!"  << std::endl;
}

std::vector<booster_vision::SegmentationRes> YoloV8SegmentorTRT::Inference(const cv::Mat& img) {
    std::vector<float> factors;
    if (!PreProcess(img, model_input_dims_.d + 2, input_buff_, factors))
    {
        return {};
    }

    // Memcpy from host input buffers to device input buffers
    MemcpyBuffers(input_mem_,input_buff_, input_size_ * sizeof(float),cudaMemcpyHostToDevice, async_infer_, stream_);

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
    MemcpyBuffers(det_output_buff_, det_output_mem_, det_output_size_ * sizeof(float), cudaMemcpyDeviceToHost, async_infer_, stream_);
    MemcpyBuffers(mask_output_buff_, mask_output_mem_, mask_output_size_ * sizeof(float), cudaMemcpyDeviceToHost, async_infer_, stream_);

    if (async_infer_)
    {
        cudaStreamSynchronize(stream_);
    }

    img_width_ = img.cols;
    img_height_ = img.rows;
    std::vector<booster_vision::SegmentationRes> outputs = PostProcess(factors);

    std::cout << "finish inference " << std::endl;
    return outputs;
}

std::vector<booster_vision::SegmentationRes> YoloV8SegmentorTRT::PostProcess(std::vector<float> factors) {
    cv::Mat det_outputs(model_output1_dims_.d[1], model_output1_dims_.d[2], CV_32F, det_output_buff_);
    std::vector<cv::Mat> masks;
    const int mask_num = model_output2_dims_.d[1];
    const int mask_height = model_output2_dims_.d[2];
    const int mask_width = model_output2_dims_.d[3];
    for (int i = 0; i < mask_num; ++i) {
        cv::Mat mask(mask_height, mask_width, CV_32F, mask_output_buff_ + i * mask_height * mask_width);
        masks.push_back(mask);
    }

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<float>> masks_scores;
    // Preprocessing output results
    const int class_num = model_output1_dims_.d[1] - 4 - 32; // 4 for box[x,y,w,h], 32 for mask number
    int rows = det_outputs.size[0];
    int dimensions = det_outputs.size[1];
    bool yolov8 = false;

    if (dimensions > rows)
    {
        yolov8 = true;
        rows = det_outputs.size[1];
        dimensions = det_outputs.size[0];

        det_outputs = det_outputs.reshape(1, dimensions);
        cv::transpose(det_outputs, det_outputs);
    }

    float* data = (float*)det_outputs.data;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        cv::Mat scores(1, class_num, CV_32FC1, classes_scores);
        // std::cout << "scores: " << scores << std::endl;
        cv::Point class_id;
        double max_class_score;

        minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

        if (max_class_score > confidence_)
        {
            auto bbox = ProcessBoundingBox(data, factors, img_width_, img_height_);
            if (bbox.width <= 0 || bbox.height <= 0) continue;

            boxes.push_back(bbox);
            confidences.push_back(max_class_score);
            class_ids.push_back(class_id.x);

            std::vector<float> mask_scores;
            float* scores = data + 4 + class_num; // Skip the box and class scores
            for (int j = 0; j < mask_num; ++j) {
                mask_scores.push_back(scores[j]);
            }
            masks_scores.push_back(mask_scores);
        }

        data += dimensions;
    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, 0.25, 0.4, nms_result);

    std::vector<booster_vision::SegmentationRes> segmentations{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        booster_vision::SegmentationRes result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.bbox = boxes[idx];
        
        float mask_resize_factor = mask_width / 1280.0;
        cv::Rect resized_bbox(
            int(result.bbox.x * mask_resize_factor),
            int(result.bbox.y * mask_resize_factor),
            int(result.bbox.width * mask_resize_factor),
            int(result.bbox.height * mask_resize_factor)
        );

        // reconstruct mask
        auto mask_score = masks_scores[idx];
        cv::Mat mask = cv::Mat::zeros(mask_height, mask_width, CV_32F);
        for (int v = resized_bbox.y; v < resized_bbox.y + resized_bbox.height; ++v) {
            for (int u = resized_bbox.x; u < resized_bbox.x + resized_bbox.width; ++u) {
                float score = 0.0f;
                for (int j = 0; j < mask_num; ++j) {
                    score += masks[j].at<float>(v, u) * mask_score[j];
                }
                score =  1.0 / (1.0 + exp(-score)); // Sigmoid activation
                mask.at<float>(v, u) = score;
            }
        }
        cv::Rect mask_bbox(0, 0, int(img_width_ * mask_resize_factor), int(img_height_  * mask_resize_factor));
        cv::resize(mask(mask_bbox), result.mask, cv::Size(img_width_, img_height_), 0, 0, cv::INTER_LINEAR);
        result.mask = result.mask > 0.5; // Binarize the mask
        result.mask.convertTo(result.mask, CV_8U, 255.0); // Convert to 8-bit mask

        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(result.mask, result.contour, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
        segmentations.push_back(result);
    }

    return segmentations;
}

YoloV8SegmentorTRT::~YoloV8SegmentorTRT() {
  cudaStreamDestroy(stream_);
  cudaFree(input_mem_);
  cudaFree(det_output_mem_);
  cudaFree(mask_output_mem_);
  free(input_buff_);
  free(det_output_buff_);
  free(mask_output_buff_);
}

#endif