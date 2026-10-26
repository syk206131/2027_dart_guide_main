%% fit_roll_yaw.m
% roll→yaw 瞄准点补偿拟合脚本（流水线第 3 步，与距离/高度拟合相互独立）
%
% 模型：aim_point_px = c0 + c1*roll + c2*roll^2
%   roll         = 电控 IMU 的 roll 偏角（rad，顺时针为正，终端打印的 roll 值）
%   aim_point_px = 该 roll 下命中目标时，终端打印的 yaw 平均偏移 avg（像素）
%
% 物理含义（对应 configs/test.yaml roll_yaw_compensation_coeffs 注释）：
%   roll 顺时针 → 弹着偏右 → 需云台向左补 → 瞄准点为正值（目标需偏右），
%   瞄准判定为 yaw_offset 收敛到 aim_point_px（减法语义）。
%
% 采数流程：
%   ① 调平（roll≈0）试射定零点：调 yaw 至落点正中，记录终端 avg 作为 (roll_0, aim_0)；
%   ② 垫已知 roll 角（正/负多档，建议 ±3° 内 5 档以上）试射，每档调 yaw 至落点回正，
%      记录当时打印的 roll 与 avg；
%   ③ 本脚本二次拟合。
%
% 注意系数顺序：polyfit 输出降幂 [c2, c1, c0]，
%   yaml 的 roll_yaw_compensation_coeffs 为升幂 [c0, c1, c2]，脚本输出已排好。

clear; clc; close all

%% 1. 录入标定样本  ↓↓↓ 替换为你的实测数据（两向量等长） ↓↓↓
roll_rad  = [-0.052 -0.035 -0.017 0.000 0.017 0.035 0.052];   % 终端打印 roll（rad）
aim_px    = [ -30   -20   -10    0    10   20   30];          % 命中时终端 avg（像素）

assert(numel(roll_rad) == numel(aim_px), 'roll_rad 与 aim_px 长度必须一致');
assert(numel(roll_rad) >= 3, '二次拟合至少需要 3 个样本点');
roll_rad = roll_rad(:);
aim_px   = aim_px(:);

%% 2. 二次拟合
p = polyfit(roll_rad, aim_px, 2);    % p = [c2, c1, c0]（降幂）
c2 = p(1); c1 = p(2); c0 = p(3);

aim_fit = polyval(p, roll_rad);
resid   = aim_px - aim_fit;
ss_res  = sum(resid.^2);
ss_tot  = sum((aim_px - mean(aim_px)).^2);
r2 = 1 - ss_res / ss_tot;

%% 3. 结果输出（可直接复制到 configs/test.yaml）
fprintf('\n===== 拟合结果 =====\n');
fprintf('aim_point_px = %.6f + %.6f*roll + %.6f*roll^2\n', c0, c1, c2);
fprintf('R^2 = %.4f\n', r2);
fprintf('残差最大绝对值: %.3f px\n', max(abs(resid)));
fprintf('\n复制到 configs/test.yaml（顺序 [c0, c1, c2] 升幂）：\n');
fprintf('roll_yaw_compensation_coeffs: [%.6f, %.6f, %.6f]\n\n', c0, c1, c2);

%% 4. 可视化
x_plot = linspace(min(roll_rad), max(roll_rad), 200);

figure('Name', 'roll→yaw 瞄准点拟合', 'Position', [100 100 1100 420]);

subplot(1, 2, 1); hold on; grid on
plot(roll_rad, aim_px, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', '标定样本');
plot(x_plot, polyval(p, x_plot), 'b-', 'LineWidth', 1.5, 'DisplayName', ...
    sprintf('%.3f + %.3fr + %.3fr^2', c0, c1, c2));
xlabel('roll (rad，顺时针为正)'); ylabel('aim\_point (px)');
title(sprintf('二次拟合  R^2 = %.4f', r2));
legend('Location', 'northwest');

subplot(1, 2, 2); hold on; grid on
stem(rad2deg(roll_rad), resid, 'filled');
yline(0, 'k--');
xlabel('roll (deg)'); ylabel('残差 (px)');
title('拟合残差（应 < 瞄准死区 3px）');

%% 5. 填回后的符号自检（重要）
% 将所有系数填入 yaml 后，垫一个已知 roll 角试射：
%   若落点偏差【同方向翻倍】→ 符号反了，把三个系数全部取负重新填入；
%   若落点回正或偏差明显减小 → 符号正确。
% 原理：roll 补偿的符号约定必须与标定数据定义完全一致，否则补偿方向相反、越补越偏。
