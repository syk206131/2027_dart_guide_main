#include "dart_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace
{
template <typename T>
T require(const YAML::Node & yaml, const std::string & key)
{
  if (!yaml[key]) throw std::runtime_error("missing required config: " + key);
  return yaml[key].as<T>();
}

template <typename T>
T optional(const YAML::Node & yaml, const std::string & key, const T & fallback)
{
  if (!yaml[key]) return fallback;
  return yaml[key].as<T>();
}

/// 从 3x3 camera_matrix 提取 fy（索引 4）和 cy（索引 5）
void extract_fy_cy(const std::vector<double> & m, double & fy, double & cy)
{
  if (m.size() != 9) throw std::runtime_error("camera_matrix must contain 9 elements");
  fy = m[4];
  cy = m[5];
}
}  // namespace

DartCalculator::DartCalculator(const std::string & config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);

  // 距离修正系数（占位）
  range_coeffs_ =
    optional<std::vector<double>>(yaml, "range_correction_coeffs", {0.0, 1.0, 0.0});
  if (range_coeffs_.size() != 3)
    throw std::runtime_error("range_correction_coeffs must contain [a0, a1, a2]");

  // 高度解算系数（占位）
  height_coeffs_ =
    optional<std::vector<double>>(yaml, "height_coeffs", {0.0, 0.0, 0.0});
  if (height_coeffs_.size() != 3)
    throw std::runtime_error("height_coeffs must contain [b0, b1, b2]");

  // 左右相机归一化参数（从 camera_matrix 提取 fy, cy）
  const auto left_m =
    require<std::vector<double>>(yaml, "left_camera_matrix");
  const auto right_m =
    require<std::vector<double>>(yaml, "right_camera_matrix");
  extract_fy_cy(left_m, left_fy_, left_cy_);
  extract_fy_cy(right_m, right_fy_, right_cy_);

  // 能量系数（占位）：E = a·(m镖+m滑台)·(g/4)·S²/(cos²θ_eff·(S·tanθ_eff − Δh)) + b + c·roll²
  // θ_eff = arcsin(sinθ·cos roll)，a ≈ 1 无量纲，b 固定损耗(J)，c·roll² roll摩擦(J/rad²)
  theta_rad_ = optional<double>(yaml, "launch_theta_rad", 0.0);
  g_ = optional<double>(yaml, "gravity", 9.80665);
  sled_mass_kg_ = optional<double>(yaml, "sled_mass_kg", 0.0);
  const auto energy_coeffs =
    optional<std::vector<double>>(yaml, "energy_coeffs", {0.0, 0.0, 0.0});
  if (energy_coeffs.size() != 3)
    throw std::runtime_error("energy_coeffs must contain [a, b, c]");
  energy_a_ = energy_coeffs[0];
  energy_b_ = energy_coeffs[1];
  energy_roll_quad_ = energy_coeffs[2];

  // roll→yaw 瞄准点系数（占位）
  roll_yaw_coeffs_ =
    optional<std::vector<double>>(yaml, "roll_yaw_compensation_coeffs", {0.0, 0.0, 0.0});
  if (roll_yaw_coeffs_.size() != 3)
    throw std::runtime_error("roll_yaw_compensation_coeffs must contain [c0, c1, c2]");

  // 每发镖体质量（kg，占位）：逐发称量填入，能量公式动能项 ∝ m
  dart_masses_ =
    optional<std::vector<double>>(yaml, "dart_masses", {0.25, 0.25, 0.25, 0.25});
  if (dart_masses_.empty())
    throw std::runtime_error("dart_masses must not be empty");
}

DartCalculator::Output DartCalculator::compute(
  const StereoRangeEstimator::Result & range, const cv::Point2d & left_center,
  const cv::Point2d & right_center, float roll_rad, uint8_t dart_index) const
{
  Output out;

  // 1. 距离修正
  if (!range.valid) return out;
  const double raw = range.raw_range_m;
  const auto & rc = range_coeffs_;
  const double D = rc[0] + rc[1] * raw + rc[2] * raw * raw;
  out.distance_m = static_cast<float>(D);

  // 2. 高度解算：归一化 y 偏移平均 → H = b0 + b1*D + b2*D*v_bar
  //    针孔模型下 v_bar = (v-cy)/fy = Y/Z = tan(视线与光轴夹角)，即 v_bar 本身
  //    就是斜率而非角度，故 θ_c≈0 时 H = h_c + D*v_bar 为精确线性关系，无需三角函数；
  //    b1 吸收装配残余俯仰（应≈0），b2 吸收尺度误差（应≈1），b0≈尺量 h_c
  const double v_l_norm = (left_center.y - left_cy_) / left_fy_;
  const double v_r_norm = (right_center.y - right_cy_) / right_fy_;
  const double v_bar = (v_l_norm + v_r_norm) * 0.5;
  const auto & hc = height_coeffs_;
  const double H = hc[0] + hc[1] * D + hc[2] * D * v_bar;
  out.height_m = static_cast<float>(H);
  out.valid = true;

  // 3. roll→yaw 瞄准点
  // aim_point = c0 + c1*roll + c2*roll²
  const auto & yc = roll_yaw_coeffs_;
  const double roll = static_cast<double>(roll_rad);
  const double aim_point = yc[0] + yc[1] * roll + yc[2] * roll * roll;
  out.yaw_offset_px = static_cast<float>(aim_point);

  // 4. 能量计算：E = a·m_total·(g/4)·S²/(cos²θ_eff·(S·tanθ_eff − Δh)) + b + c·roll²
  // 弹道需求 v₀² = g·S²/(2cos²θ_eff·(S·tanθ_eff − Δh))，总动能 E_kin = ½·m_total·v₀²。
  // θ_eff = arcsin(sinθ·cos roll)：roll 对弹道几何的修正（φ² 量级，免费带上）。
  // 动能分配：滑轨上滑台与镖一起加速，分离瞬间同速，镖带走 m镖/m_total 份额。
  // a 无量纲（≈1，拟合吸收弹簧效率与比例损耗），b 固定损耗(J)，c·roll² roll摩擦(J/rad²)。
  // 弹簧压缩量换算由电控完成，视觉不涉及弹簧系数 k。
  const double m_dart =
    (dart_index < dart_masses_.size()) ? dart_masses_[dart_index] : dart_masses_[0];
  const double m_total = m_dart + sled_mass_kg_;
  const double S = D;
  const double dh = H;
  const double roll_abs = std::fabs(roll);
  const double sin_eff = std::sin(theta_rad_) * std::cos(roll_abs);
  const double theta_eff = std::asin(std::clamp(sin_eff, -1.0, 1.0));
  const double cos_t = std::cos(theta_eff);
  const double tan_t = std::tan(theta_eff);
  const double denom = cos_t * cos_t * (S * tan_t - dh);
  double energy = energy_b_ + energy_roll_quad_ * roll_abs * roll_abs;
  if (std::abs(denom) > 1e-6)
    energy += energy_a_ * m_total * (g_ * 0.25) * S * S / denom;
  out.energy_j = static_cast<float>(std::max(0.0, energy));

  return out;
}
