#ifndef TASKS__DART__DART_CALCULATOR_HPP_
#define TASKS__DART__DART_CALCULATOR_HPP_

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tasks/stereo/stereo_range.hpp"

/// 飞镖弹道解算模块：距离修正、高度解算、拉力计算、roll→yaw 补偿
/// 所有拟合系数通过 yaml 配置加载，MATLAB 标定后填入。
class DartCalculator
{
public:
  struct Output
  {
    float distance_m = -1.0f;      // 修正后的水平距离（无效为 -1）
    float height_m = 0.0f;         // 目标相对高度（相机平面到出镖点高度差，m）
    float energy_j = 0.0f;         // 所需总动能（滑台+镖体，单位 J，标定后有效）
    float yaw_offset_px = 0.0f;    // roll→yaw 瞄准点（像素）：瞄准时 yaw_offset 应等于此值。
    bool valid = false;            // 距离和高度是否都有效
  };

  explicit DartCalculator(const std::string & config_path);

  /// 一站式计算
  /// @param range          双目测距结果（raw + 修正）
  /// @param left_center    左相机目标质心（像素）
  /// @param right_center   右相机目标质心（像素）
  /// @param roll_rad       电控发送的 roll 偏角（顺时针为正，rad）
  /// @param dart_index     当前镖体序号（0~3），用于叠加每发基础 yaw 偏移
  Output compute(
    const StereoRangeEstimator::Result & range, const cv::Point2d & left_center,
    const cv::Point2d & right_center, float roll_rad, uint8_t dart_index = 0) const;

private:
  // ── 距离修正 ──
  // [a0, a1, a2]: range = a0 + a1*raw + a2*raw^2
  std::vector<double> range_coeffs_;

  // ── 高度解算 ──
  // [b0, b1, b2]: H = b0 + b1*D + b2*D*v_bar（v_bar 本身即夹角正切，精确线性）
  // v_bar = 0.5*( (v_l-cy_l)/fy_l + (v_r-cy_r)/fy_r )，左右归一化后取平均
  std::vector<double> height_coeffs_;
  double left_cy_ = 0.0, left_fy_ = 1.0;
  double right_cy_ = 0.0, right_fy_ = 1.0;

  // ── 能量计算（视觉→电控直接发送所需总动能，单位 J） ──
  // E = a·(m镖+m滑台)·(g/4)·S²/(cos²θ_eff·(S·tanθ_eff − Δh)) + b + c·roll²
  // 由弹道需求 v₀² = g·S²/(2cos²θ_eff·(S·tanθ_eff − Δh)) 与 E_kin = ½·m_total·v₀² 推出；
  // θ_eff = arcsin(sinθ·cos roll)，θ = theta_rad_（出射仰角设计值），g = g_（当地重力），
  // a 无量纲（理想值≈1，拟合自动吸收弹簧效率与比例损耗），b = 固定损耗（J），
  // c·roll² 吸收 roll 摩擦变化（J/rad²），
  // m镖 = dart_masses_[dart_index]，m滑台 = sled_mass_kg_（滑台带镖一起加速，分离后镖飞出）。
  // 弹簧压缩量换算（E_弹簧 = E/效率，x = sqrt(2·E_弹簧/k)）由电控完成，视觉不涉及 k。
  double theta_rad_ = 0.0;
  double g_ = 9.80665;
  double energy_a_ = 0.0;
  double energy_b_ = 0.0;
  double energy_roll_quad_ = 0.0;
  double sled_mass_kg_ = 0.0;  // 滑台质量（kg），动能分配分母 = m镖 + m滑台

  // ── 每发镖体质量（kg）──
  // 能量公式动能项 ∝ 质量，按 dart_index 取对应质量修正。
  // 标定：逐发称量填入。
  std::vector<double> dart_masses_;

  // ── roll→yaw 瞄准点 ──
  // [c0, c1, c2]: aim_point_px = c0 + c1*roll + c2*roll^2
  // 语义：roll 为 roll_rad 时，yaw_offset 收敛到 aim_point_px 才判定瞄准
  std::vector<double> roll_yaw_coeffs_;
};

#endif  // TASKS__DART__DART_CALCULATOR_HPP_
