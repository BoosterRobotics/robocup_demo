#include "booster_vision/model/onnx/segmentation_impl.h"

#include <regex>
#include <stdexcept>
#include <chrono>

namespace booster_vision {

// Reuse PreProcess and BlobFromImage from detection_impl.cpp
cv::Mat PreProcessSeg(const cv::Mat &src_img, cv::Size &dst_img_size) {
    cv::Mat dst_img;
    if (src_img.channels() == 3) {
        dst_img = src_img.clone();
        cv::cvtColor(dst_img, dst_img, cv::COLOR_BGR2RGB);
    } else {
        cv::cvtColor(src_img, dst_img, cv::COLOR_GRAY2RGB);
    }

    if (src_img.cols >= src_img.rows) {
        auto resizeScales = src_img.cols / (float)dst_img_size.width;
        cv::resize(dst_img, dst_img, cv::Size(dst_img_size.width, int(src_img.rows / resizeScales)));
    } else {
        auto resizeScales = src_img.rows / (float)dst_img_size.width;
        cv::resize(dst_img, dst_img, cv::Size(int(src_img.cols / resizeScales), dst_img_size.height));
    }
    cv::Mat tempImg = cv::Mat::zeros(dst_img_size, CV_8UC3);
    dst_img.copyTo(tempImg(cv::Rect(0, 0, dst_img.cols, dst_img.rows)));
    dst_img = tempImg;

    return dst_img;
}

template <typename T>
void BlobFromImageSeg(cv::Mat &iImg, T &iBlob) {
    int channels = iImg.channels();
    int imgHeight = iImg.rows;
    int imgWidth = iImg.cols;

    for (int c = 0; c < channels; c++) {
        for (int h = 0; h < imgHeight; h++) {
            for (int w = 0; w < imgWidth; w++) {
                iBlob[c * imgWidth * imgHeight + h * imgWidth + w] = typename std::remove_pointer<T>::type(
                    (iImg.at<cv::Vec3b>(h, w)[c]) / 255.0f);
            }
        }
    }
}

template <typename T>
Ort::Value PrepareInputTensorSeg(const cv::Mat &img, cv::Size input_size, void *data_buffer) {
    cv::Mat processed_img = PreProcessSeg(img, input_size);
    std::vector<int64_t> input_node_dims = {1, 3, input_size.width, input_size.height};

    T *data_buffer_converted = reinterpret_cast<T *>(data_buffer);
    BlobFromImageSeg(processed_img, data_buffer_converted);

    Ort::Value input_tensor = Ort::Value::CreateTensor<T>(
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU), data_buffer_converted,
        3 * input_size.area(), input_node_dims.data(), input_node_dims.size());
    return input_tensor;
}

YoloV8SegmentorONNX::~YoloV8SegmentorONNX() {
    if (data_buffer_) {
        if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            delete[] reinterpret_cast<float *>(data_buffer_);
        } else {
            delete[] reinterpret_cast<Ort::Float16_t *>(data_buffer_);
        }
        data_buffer_ = nullptr;
    }
}

void YoloV8SegmentorONNX::Init(std::string model_path) {
    std::regex pattern("[\u4e00-\u9fa5]");
    bool result = std::regex_search(model_path, pattern);
    if (result) {
        throw std::runtime_error("model path is error. Change your model path without chinese characters");
    }
    try {
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YoloSeg");
        Ort::SessionOptions session_option;
        session_option.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_option.SetIntraOpNumThreads(1);
        session_option.SetLogSeverityLevel(3);

        // Try to enable hardware acceleration providers in order: OpenVINO (Intel GPU) > CUDA > CPU
        std::vector<std::string> available_providers = Ort::GetAvailableProviders();
        std::cout << "Available ONNX Runtime Providers: ";
        for (const auto &provider : available_providers) {
            std::cout << provider << " ";
        }
        std::cout << std::endl;
        bool provider_added = false;
        
        // Try OpenVINO for Intel integrated GPU
        if (std::find(available_providers.begin(), available_providers.end(), "OpenVINOExecutionProvider") != available_providers.end()) {
            try {
                OrtOpenVINOProviderOptions openvino_options;
                openvino_options.device_type = "GPU_FP32";  // Use Intel GPU with FP32 precision
                // openvino_options.device_type = "GPU_FP16";  // Uncomment for FP16 (faster but less precise)
                session_option.AppendExecutionProvider_OpenVINO(openvino_options);
                std::cout << "[Segmentation] OpenVINO Execution Provider enabled (Intel GPU)" << std::endl;
                provider_added = true;
            } catch (const std::exception &e) {
                std::cout << "[Segmentation] OpenVINO EP failed: " << e.what() << ", falling back..." << std::endl;
            }
        }
        
        // Try CUDA for NVIDIA GPU
        if (!provider_added && std::find(available_providers.begin(), available_providers.end(), "CUDAExecutionProvider") != available_providers.end()) {
            try {
                OrtCUDAProviderOptions cuda_options;
                cuda_options.device_id = 0;
                session_option.AppendExecutionProvider_CUDA(cuda_options);
                std::cout << "[Segmentation] CUDA Execution Provider enabled (NVIDIA GPU)" << std::endl;
                provider_added = true;
            } catch (const std::exception &e) {
                std::cout << "[Segmentation] CUDA EP failed: " << e.what() << ", falling back to CPU" << std::endl;
            }
        }
        
        if (!provider_added) {
            std::cout << "[Segmentation] Using CPU Execution Provider" << std::endl;
        }

        session_ = std::make_shared<Ort::Session>(env_, model_path.c_str(), session_option);

        // Get input shape from model
        auto input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (input_shape.size() == 4) {
            model_input_size_ = {static_cast<int>(input_shape[3]), static_cast<int>(input_shape[2])};
            std::cout << "[Segmentation] Model input size: " << model_input_size_.width << "x" << model_input_size_.height << std::endl;
        }

        element_type_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetElementType();
        // allocate data buffer based on input tensor element type
        if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            std::cout << "element type: float" << std::endl;
            data_buffer_ = new float[model_input_size_.area() * 3];
        } else if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
            std::cout << "element type: float16" << std::endl;
            data_buffer_ = new Ort::Float16_t[model_input_size_.area() * 3];
        } else {
            throw std::runtime_error("element type not supported");
        }

        Ort::AllocatorWithDefaultOptions allocator;
        size_t input_num = session_->GetInputCount();
        for (size_t i = 0; i < input_num; i++) {
            Ort::AllocatedStringPtr input_node_name = session_->GetInputNameAllocated(i, allocator);
            char *temp_buf = new char[50];
            strcpy(temp_buf, input_node_name.get());
            input_node_names_.push_back(temp_buf);
        }
        size_t output_num = session_->GetOutputCount();
        for (size_t i = 0; i < output_num; i++) {
            Ort::AllocatedStringPtr output_node_name = session_->GetOutputNameAllocated(i, allocator);
            char *temp_buf = new char[10];
            strcpy(temp_buf, output_node_name.get());
            output_node_names_.push_back(temp_buf);
        }

        options_ = Ort::RunOptions{nullptr};

        // warmup
        auto start = std::chrono::high_resolution_clock::now();
        Ort::Value input_tensor(nullptr);
        cv::Mat img = cv::Mat(model_input_size_, CV_8UC3);
        if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            input_tensor = PrepareInputTensorSeg<float>(img, model_input_size_, data_buffer_);
        } else {
            input_tensor = PrepareInputTensorSeg<Ort::Float16_t>(img, model_input_size_, data_buffer_);
        }
        for (int i = 0; i < 5; i++) {
            auto output_tensors = session_->Run(options_, input_node_names_.data(), &input_tensor, 1, output_node_names_.data(),
                                                output_node_names_.size());
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "segmentation warmup takes: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    } catch (const std::exception &e) {
        throw std::runtime_error("segmentation onnx model init failed: " + std::string(e.what()));
    }
}

template <typename T>
std::vector<booster_vision::SegmentationRes> YoloV8SegmentorONNX::InferenceImpl(const cv::Mat &img) {
    int img_width = img.cols;
    int img_height = img.rows;
    
    // Prepare input tensor
    Ort::Value input_tensor = PrepareInputTensorSeg<T>(img, model_input_size_, data_buffer_);

    // Run inference
    auto start = std::chrono::high_resolution_clock::now();
    auto output_tensors = session_->Run(options_, input_node_names_.data(), &input_tensor, 1, 
                                        output_node_names_.data(), output_node_names_.size());
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "segmentation inference takes: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    
    // YOLOv8-seg outputs: 
    // output[0]: detection output [1, num_classes + 4 + 32, num_anchors]
    // output[1]: proto masks [1, 32, mask_h, mask_w]
    
    // Get detection output
    Ort::TypeInfo det_type_info = output_tensors[0].GetTypeInfo();
    auto det_tensor_info = det_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> det_output_dims = det_tensor_info.GetShape();
    auto det_output = output_tensors[0].GetTensorMutableData<typename std::remove_pointer<T>::type>();
    
    int num_channels = det_output_dims[1];  // num_classes + 4 + 32
    int num_anchors = det_output_dims[2];
    int num_classes = num_channels - 4 - 32;  // subtract bbox(4) and mask_coeffs(32)
    
    // Get proto output
    Ort::TypeInfo proto_type_info = output_tensors[1].GetTypeInfo();
    auto proto_tensor_info = proto_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> proto_output_dims = proto_tensor_info.GetShape();
    auto proto_output = output_tensors[1].GetTensorMutableData<typename std::remove_pointer<T>::type>();
    
    int mask_height = proto_output_dims[2];
    int mask_width = proto_output_dims[3];
    
    // Convert detection output to cv::Mat for easier processing
    cv::Mat raw_data;
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        raw_data = cv::Mat(num_channels, num_anchors, CV_16F, det_output);
        raw_data.convertTo(raw_data, CV_32F);
    } else {
        raw_data = cv::Mat(num_channels, num_anchors, CV_32F, det_output);
    }
    raw_data = raw_data.t();  // Transpose to [num_anchors, num_channels]
    
    // Convert proto output
    cv::Mat proto_data;
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        proto_data = cv::Mat(32, mask_height * mask_width, CV_16F, proto_output);
        proto_data.convertTo(proto_data, CV_32F);
    } else {
        proto_data = cv::Mat(32, mask_height * mask_width, CV_32F, proto_output);
    }
    
    // Extract detections
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<float>> mask_coeffs;
    
    float *data = (float *)raw_data.data;
    float resize_scale = std::max(img_width, img_height) / (float)model_input_size_.width;
    
    for (int i = 0; i < num_anchors; ++i) {
        float *row = data + i * num_channels;
        
        // Get class scores (after bbox coordinates)
        float *class_scores = row + 4;
        cv::Mat scores(1, num_classes, CV_32FC1, class_scores);
        cv::Point class_id;
        double max_class_score;
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
        
        if (max_class_score > confidence_) {
            // Extract bbox
            float x = row[0];
            float y = row[1];
            float w = row[2];
            float h = row[3];
            
            // Convert from center format to corner format
            int left = int((x - 0.5 * w) * resize_scale);
            int top = int((y - 0.5 * h) * resize_scale);
            int width = int(w * resize_scale);
            int height = int(h * resize_scale);
            
            // Clamp to image boundaries
            left = std::max(0, std::min(left, img_width - 1));
            top = std::max(0, std::min(top, img_height - 1));
            int right = std::min(img_width, left + width);
            int bottom = std::min(img_height, top + height);
            width = right - left;
            height = bottom - top;
            
            if (width >= 3 && height >= 3) {
                boxes.push_back(cv::Rect(left, top, width, height));
                confidences.push_back(max_class_score);
                class_ids.push_back(class_id.x);
                
                // Extract mask coefficients
                std::vector<float> coeffs;
                for (int j = 0; j < 32; j++) {
                    coeffs.push_back(row[4 + num_classes + j]);
                }
                mask_coeffs.push_back(coeffs);
            }
        }
    }
    
    // Apply NMS
    std::vector<int> nms_results;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_, nms_threshold_, nms_results);
    
    // Generate final results with masks
    std::vector<booster_vision::SegmentationRes> results;
    float *proto_ptr = (float *)proto_data.data;
    
    for (int idx : nms_results) {
        booster_vision::SegmentationRes result;
        result.bbox = boxes[idx];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.class_name = YoloV8Segmentor::kClassLabels[class_ids[idx]];
        
        // Generate mask from coefficients and prototypes
        cv::Mat mask_mat = cv::Mat::zeros(mask_height, mask_width, CV_32FC1);
        
        // Matrix multiplication: mask = sigmoid(coeffs @ proto)
        for (int y = 0; y < mask_height; y++) {
            for (int x = 0; x < mask_width; x++) {
                float val = 0.0f;
                for (int c = 0; c < 32; c++) {
                    val += mask_coeffs[idx][c] * proto_ptr[c * mask_height * mask_width + y * mask_width + x];
                }
                // Apply sigmoid
                val = 1.0f / (1.0f + std::exp(-val));
                mask_mat.at<float>(y, x) = val;
            }
        }
        
        // Resize mask to input size
        cv::resize(mask_mat, mask_mat, model_input_size_, 0, 0, cv::INTER_LINEAR);
        
        // Crop mask to detection bbox (in input image coordinates)
        int mask_left = int(boxes[idx].x / resize_scale);
        int mask_top = int(boxes[idx].y / resize_scale);
        int mask_right = int((boxes[idx].x + boxes[idx].width) / resize_scale);
        int mask_bottom = int((boxes[idx].y + boxes[idx].height) / resize_scale);
        
        mask_left = std::max(0, std::min(mask_left, model_input_size_.width - 1));
        mask_top = std::max(0, std::min(mask_top, model_input_size_.height - 1));
        mask_right = std::min(model_input_size_.width, mask_right);
        mask_bottom = std::min(model_input_size_.height, mask_bottom);
        
        cv::Rect mask_roi(mask_left, mask_top, mask_right - mask_left, mask_bottom - mask_top);
        if (mask_roi.width > 0 && mask_roi.height > 0) {
            cv::Mat cropped_mask = mask_mat(mask_roi);
            
            // Resize to original bbox size
            cv::Mat final_mask = cv::Mat::zeros(boxes[idx].height, boxes[idx].width, CV_32FC1);
            cv::resize(cropped_mask, final_mask, final_mask.size(), 0, 0, cv::INTER_LINEAR);
            
            // Threshold mask
            cv::Mat binary_mask;
            cv::threshold(final_mask, binary_mask, 0.5, 255, cv::THRESH_BINARY);
            binary_mask.convertTo(binary_mask, CV_8UC1);
            
            // Create full-size mask
            result.mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
            binary_mask.copyTo(result.mask(boxes[idx]));
            
            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // Adjust contour coordinates to absolute image coordinates
            for (auto& contour : contours) {
                for (auto& pt : contour) {
                    pt.x += boxes[idx].x;
                    pt.y += boxes[idx].y;
                }
            }
            result.contour = contours;
        } else {
            result.mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
        }
        
        results.push_back(result);
    }
    
    end = std::chrono::high_resolution_clock::now();
    std::cout << "segmentation post process takes: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    
    return results;
}

std::vector<booster_vision::SegmentationRes> YoloV8SegmentorONNX::Inference(const cv::Mat &img) {
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        return InferenceImpl<float>(img);
    } else {
        return InferenceImpl<Ort::Float16_t>(img);
    }
}

} // namespace booster_vision
