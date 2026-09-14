#ifndef TASKS__LIGHT_DETECT__DETECT_HPP_
#define TASKS__LIGHT_DETECT__DETECT_HPP_
#include "openvino/openvinoInfer.hpp"
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <memory>
class LightDetect {
public:
explicit LightDetect(const std::string &config_path);
    ~LightDetect() = default;
    std::vector<OpenvinoInfer::Light> detect(cv::Mat &src, const cv::Size2d &dst_size, const int &my_color, const bool &startup);
private:
    std::unique_ptr<OpenvinoInfer> openvino_infer_;
};
#endif  // TASKS__LIGHT_DETECT__DETECT_HPP_
