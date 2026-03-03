#pragma once
#include <string>
#include <NvInfer.h>
#include <random>
#include "booster_vision/model/detector.h"
#include "booster_vision/model/segmentor.h"

using namespace nvinfer1;

#if (NV_TENSORRT_MAJOR == 8) && (NV_TENSORRT_MINOR == 6)

#include "booster_vision/model//trt/config.h"
void serialize_det_engine(std::string& wts_name, std::string& engine_name, int& is_p, std::string& sub_type, float& gd,
                      float& gw, int& max_channels);
void deserialize_det_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context);

void prepare_det_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_buffer_host, float** decode_ptr_host, float** decode_ptr_device,
                    std::string cuda_post_process);

void infer_det(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
           float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process,
           float confidence_threshold = kConfThresh, float nms_threhosld = kNmsThresh);

void serialize_seg_engine(std::string& wts_name, std::string& engine_name, std::string& sub_type, float& gd, float& gw,
                      int& max_channels);

void deserialize_seg_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context);

void prepare_seg_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_seg_buffer_device, float** output_buffer_host, float** output_seg_buffer_host,
                    float** decode_ptr_host, float** decode_ptr_device, std::string cuda_post_process);

void infer_seg(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, float* output_seg,
           int batchsize, float* decode_ptr_host, float* decode_ptr_device, int model_bboxes,
           std::string cuda_post_process);

class YoloV8DetectorTRT : public booster_vision::YoloV8Detector {
 public:
  YoloV8DetectorTRT(const std::string& path) : booster_vision::YoloV8Detector(path) {
    Init(path);
  }
  ~YoloV8DetectorTRT();

  void Init(std::string model_path) override;
  std::vector<booster_vision::DetectionRes> Inference(const cv::Mat& img) override;

 private:
  IRuntime* runtime = nullptr;
  ICudaEngine* engine = nullptr;
  IExecutionContext* context = nullptr;

  // Prepare cpu and gpu buffers
  int model_bboxes;
  cudaStream_t stream;
  float* device_buffers[2];
  float* output_buffer_host = nullptr;
  float* decode_ptr_host = nullptr;
  float* decode_ptr_device = nullptr;
  std::string cuda_post_process = "g";
};

class YoloV8SegmentorTRT : public booster_vision::YoloV8Segmentor {
  public:
    YoloV8SegmentorTRT(const std::string& name, const float& conf) : booster_vision::YoloV8Segmentor(name, conf) {
      Init(name);
    }
    ~YoloV8SegmentorTRT() override;
  
    void Init(std::string model_path);
    std::vector<booster_vision::SegmentationRes> Inference(const cv::Mat& img) override;
  
  private:
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
  
    // Prepare cpu and gpu buffers
    int model_bboxes;
    cudaStream_t stream;
    float* device_buffers[3];
    float* output_buffer_host = nullptr;
    float* output_seg_buffer_host = nullptr;
    float* decode_ptr_host = nullptr;
    float* decode_ptr_device = nullptr;
    std::string cuda_post_process = "c";
};

#elif (NV_TENSORRT_MAJOR == 10) && (NV_TENSORRT_MINOR == 3)

struct InferDeleter
{
	template <typename T>
	void operator()(T* obj) const
	{
		delete obj;
	}
};

class YoloV8DetectorTRT : public booster_vision::YoloV8Detector {
 public:
  YoloV8DetectorTRT(const std::string& path) : booster_vision::YoloV8Detector(path) {
    Init(path);
  }
  ~YoloV8DetectorTRT();

  void Init(std::string model_path) override;
  std::vector<booster_vision::DetectionRes> Inference(const cv::Mat& img) override;

 private:
	bool LoadEngine();

	std::vector<booster_vision::DetectionRes> PostProcess(std::vector<float> factors);

	std::shared_ptr<nvinfer1::IRuntime> runtime_;
	std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
	cudaStream_t stream_ = 0;
	
	nvinfer1::Dims model_input_dims_;
	nvinfer1::Dims model_output_dims_;

  bool async_infer_ = true;
	size_t input_size_;
	size_t output_size_;
  int img_width_ = 0;
  int img_height_ = 0;
	std::vector<void*> bindings_;
	void* input_mem_{ nullptr };
	void* output_mem_{ nullptr };

	float* input_buff_;
	float* output_buff_;

	bool squre_input_ = true;
};

class YoloV8SegmentorTRT : public booster_vision::YoloV8Segmentor {
 public:
	YoloV8SegmentorTRT(const std::string& path, const float& conf) : booster_vision::YoloV8Segmentor(path, conf) {
		Init(path);
	}
	~YoloV8SegmentorTRT();

	void Init(std::string model_path);
	std::vector<booster_vision::SegmentationRes> Inference(const cv::Mat& img) override;

 private:
	bool LoadEngine();
	std::vector<booster_vision::SegmentationRes> PostProcess(std::vector<float> factors);

	std::shared_ptr<nvinfer1::IRuntime> runtime_;
	std::shared_ptr<nvinfer1::ICudaEngine> engine_;
	std::shared_ptr<nvinfer1::IExecutionContext> context_;
	cudaStream_t stream_ = 0;

	nvinfer1::Dims model_input_dims_;
	nvinfer1::Dims model_output1_dims_;  // detection output
	nvinfer1::Dims model_output2_dims_;  // mask output

	bool async_infer_ = true;
	size_t input_size_;
	size_t det_output_size_;
	size_t mask_output_size_;
	int img_width_ = 0;
	int img_height_ = 0;
	std::vector<void*> bindings_;
	void* input_mem_{ nullptr };
	void* det_output_mem_{ nullptr };
	void* mask_output_mem_{ nullptr };

	float* input_buff_;
	float* det_output_buff_;
	float* mask_output_buff_;

	bool squre_input_ = true;
};

#endif