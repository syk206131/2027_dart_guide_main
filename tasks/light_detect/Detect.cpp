#include "Detect.hpp"

LightDetect::LightDetect(const std::string &config_path) {
    this->openvino_infer_ = std::make_unique<OpenvinoInfer>(config_path);
}

std::vector<OpenvinoInfer::Light> LightDetect::detect(cv::Mat &src, const cv::Size2d &dst_size, const int &my_color, const bool &startup) {
    auto lights = this->openvino_infer_->infer(src, dst_size, my_color, startup);
    // The network works on the letterboxed image. Convert detections back to
    // the source-camera pixel coordinate system before ROI refinement and
    // stereo triangulation.
    OpenvinoInfer::fitRec(lights, cv::Size2d(src.cols, src.rows), dst_size);
    return lights;
}
