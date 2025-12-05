#include "booster_vision/model/onnx/detection_impl.h"

#include <regex>
#include <stdexcept>
#include <chrono>

namespace booster_vision {

cv::Mat PreProcess(const cv::Mat &src_img, cv::Size &dst_img_size) {
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

    // cv::resize(src_img, dst_img, dst_img_size);
    return dst_img;
}

template <typename T>
void BlobFromImage(cv::Mat &iImg, T &iBlob) {
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
Ort::Value PrepareInputTensor(const cv::Mat &img, cv::Size input_size, void *data_buffer) {
    cv::Mat processed_img = PreProcess(img, input_size);
    std::vector<int64_t> input_node_dims = {1, 3, input_size.width, input_size.height};

    T *data_buffer_converted = reinterpret_cast<T *>(data_buffer);
    BlobFromImage(processed_img, data_buffer_converted);

    Ort::Value input_tensor = Ort::Value::CreateTensor<T>(
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU), data_buffer_converted,
        3 * input_size.area(), input_node_dims.data(), input_node_dims.size());
    return input_tensor;
}

YoloV8DetectorONNX::~YoloV8DetectorONNX() {
    if (data_buffer_) {
        if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            delete[] reinterpret_cast<float *>(data_buffer_);
        } else {
            delete[] reinterpret_cast<Ort::Float16_t *>(data_buffer_);
        }
        data_buffer_ = nullptr;
    }
}

void YoloV8DetectorONNX::Init(std::string model_path) {
    std::regex pattern("[\u4e00-\u9fa5]");
    bool result = std::regex_search(model_path, pattern);
    if (result) {
        throw std::runtime_error("model path is error.Change your model path without chinese characters");
    }
    try {
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Yolo");
        Ort::SessionOptions session_option;
        session_option.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_option.SetIntraOpNumThreads(1);
        session_option.SetLogSeverityLevel(3);

        session_ = std::make_shared<Ort::Session>(env_, model_path.c_str(), session_option);

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
            input_tensor = PrepareInputTensor<float>(img, model_input_size_, data_buffer_);
        } else {
            input_tensor = PrepareInputTensor<Ort::Float16_t>(img, model_input_size_, data_buffer_);
        }
        for (int i = 0; i < 5; i++) {
            auto output_tensors = session_->Run(options_, input_node_names_.data(), &input_tensor, 1, output_node_names_.data(),
                                                output_node_names_.size());
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "warmup takes: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    } catch (const std::exception &e) {
        throw std::runtime_error("detection onnx model init failed: " + std::string(e.what()));
    }
}

template <typename T>
std::vector<booster_vision::DetectionRes> YoloV8DetectorONNX::InferenceImpl(const cv::Mat &img) {
    int img_width = img.cols;
    int img_height = img.rows;
    Ort::Value input_tensor = PrepareInputTensor<T>(img, model_input_size_, data_buffer_);

    auto start = std::chrono::high_resolution_clock::now();
    auto output_tensor = session_->Run(options_, input_node_names_.data(), &input_tensor, 1, output_node_names_.data(),
                                       output_node_names_.size());
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "inference takes: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    Ort::TypeInfo type_info = output_tensor.front().GetTypeInfo();
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> output_node_dims = tensor_info.GetShape();
    auto output = output_tensor.front().GetTensorMutableData<typename std::remove_pointer<T>::type>();

    int signal_res_num = output_node_dims[1];
    int stride_num = output_node_dims[2];
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    cv::Mat raw_data;

    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        raw_data = cv::Mat(signal_res_num, stride_num, CV_16F, output);
        raw_data.convertTo(raw_data, CV_32F);
    } else {
        raw_data = cv::Mat(signal_res_num, stride_num, CV_32F, output);
    }
    raw_data = raw_data.t();

    float *data = (float *)raw_data.data;

    for (int i = 0; i < stride_num; ++i) {
        float *classesScores = data + 4;
        // TODO: use normal member instead of static number
        cv::Mat scores(1, YoloV8Detector::kClassLabels.size(), CV_32FC1, classesScores);
        cv::Point class_id;
        double max_class_score;
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
        if (max_class_score > confidence_) {
            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            // TODO: fix magic number
            int resize_scales = 2;
            int left = int((x - 0.5 * w) * resize_scales);
            int top = int((y - 0.5 * h) * resize_scales);

            int width = int(w * resize_scales);
            int height = int(h * resize_scales);

            if (left < 0 || left > img_width - 1) continue;
            if (top < 0 || top > img_height - 1) continue;

            int right = std::min(img_width - 1, left + width);
            int bottom = std::min(img_height - 1, top + height);
            width = right - left;
            height = bottom - top;
            if (width < 3 || height < 3) continue;

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(max_class_score);
            class_ids.push_back(class_id.x);
        }
        data += signal_res_num;
    }
    std::vector<int> nms_results;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_, nms_threshold_, nms_results);
    std::vector<booster_vision::DetectionRes> ret;
    for (int i = 0; i < nms_results.size(); ++i) {
        int idx = nms_results[i];
        booster_vision::DetectionRes result;
        result.bbox = boxes[idx];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.class_name = YoloV8Detector::kClassLabels[class_ids[idx]];
        ret.push_back(result);
    }
    end = std::chrono::high_resolution_clock::now();
    std::cout << "post process takes: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;
    return ret;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorONNX::Inference(const cv::Mat &img) {
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        return InferenceImpl<float>(img);
    } else {
        return InferenceImpl<Ort::Float16_t>(img);
    }
}

} // namespace booster_vision