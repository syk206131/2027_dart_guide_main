#include "Detect.hpp"

#include "tools/yaml.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
template <typename T>
T read_or(const YAML::Node &yaml, const std::string &key, const T &fallback)
{
    return yaml[key] ? yaml[key].as<T>() : fallback;
}

cv::Scalar read_hsv_or(const YAML::Node &yaml, const std::string &key, const cv::Scalar &fallback)
{
    if (!yaml[key]) return fallback;
    auto v = yaml[key].as<std::vector<int>>();
    if (v.size() != 3) {
        tools::logger()->error("[YAML] {} must contain [H, S, V]!", key);
        exit(1);
    }
    return cv::Scalar(v[0], v[1], v[2]);
}
}  // namespace

LightDetect::LightDetect(const std::string &config_path) {
    const auto yaml = tools::load(config_path);
    this->hsv_lower_ = read_hsv_or(yaml, "green_lower", cv::Scalar(35, 100, 100));
    this->hsv_upper_ = read_hsv_or(yaml, "green_upper", cv::Scalar(85, 255, 255));
    // 最小轮廓面积 px²
    this->min_area_ = read_or<double>(yaml, "min_light_area", 30.0);   
    // 最大轮廓面积占比
    this->max_area_ratio_ = read_or<double>(yaml, "max_light_area_ratio", 0.05);
    // 最小圆度
    this->min_circularity_ = read_or<double>(yaml, "min_circularity", 0.6);
    // 最小外接矩形宽高比
    this->min_aspect_ratio_ = read_or<double>(yaml, "min_aspect_ratio", 0.6);
    // 外接矩形宽高比范围
    this->max_aspect_ratio_ = read_or<double>(yaml, "max_aspect_ratio", 1.67);
}

std::vector<LightDetect::Light> LightDetect::detect(cv::Mat &src) {
    std::vector<Light> result;

    cv::Mat blurred, hsv, mask;
    cv::GaussianBlur(src, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, this->hsv_lower_, this->hsv_upper_, mask);

    // 开运算去孤立噪点，闭运算补灯珠内部空洞
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double max_area = this->max_area_ratio_ * src.cols * src.rows;

    for (const auto &contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < this->min_area_ || area > max_area) continue;

        const double perimeter = cv::arcLength(contour, true);
        if (perimeter < 1e-5) continue;
        const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (circularity < this->min_circularity_) continue;

        const cv::Rect box = cv::boundingRect(contour);
        const double aspect_ratio =
            static_cast<double>(box.width) / static_cast<double>(box.height);
        if (aspect_ratio < this->min_aspect_ratio_ || aspect_ratio > this->max_aspect_ratio_)
            continue;

        const auto moments = cv::moments(contour);
        if (moments.m00 <= 1e-5) continue;

        Light light;
        light.id = 8;  // 保持与原网络输出中引导灯类别 id 一致
        light.score = circularity;
        light.area = area;
        light.box = cv::Rect2d(box.x, box.y, box.width, box.height);
        light.center_point = cv::Point2d(moments.m10 / moments.m00, moments.m01 / moments.m00);
        result.emplace_back(light);
    }

    // 置信度（圆度）高的排在前面
    std::sort(result.begin(), result.end(),
              [](const Light &a, const Light &b) { return a.score > b.score; });

    return result;
}
