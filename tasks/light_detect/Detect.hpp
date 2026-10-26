#ifndef TASKS__LIGHT_DETECT__DETECT_HPP_
#define TASKS__LIGHT_DETECT__DETECT_HPP_
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

// 传统视觉识别绿色圆形引导灯：HSV 阈值分割 + 形态学去噪 + 轮廓圆度筛选。
class LightDetect {
public:
    struct Light {
        int id = -1;
        double score = 0.;
        double area = 0.;  // 轮廓面积 (px^2)，双目标决策用
        cv::Rect2d box;
        cv::Point2d center_point;
    };

    explicit LightDetect(const std::string &config_path);
    ~LightDetect() = default;

    std::vector<Light> detect(cv::Mat &src);

private:
    cv::Scalar hsv_lower_;
    cv::Scalar hsv_upper_;
    double min_area_;            // 最小轮廓面积 (px^2)
    double max_area_ratio_;      // 最大轮廓面积占整帧比例
    double min_circularity_;     // 最小圆度 4*pi*S/P^2，正圆为 1
    double min_aspect_ratio_;    // 外接矩形宽/高最小值
    double max_aspect_ratio_;    // 外接矩形宽/高最大值
};
#endif  // TASKS__LIGHT_DETECT__DETECT_HPP_
