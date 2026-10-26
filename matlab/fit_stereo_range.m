%% fit_stereo_range.m
% 双目测距修正拟合脚本
%
% 模型：D = a0 + a1*raw + a2*raw^2
%   raw = 视觉原始计算距离（理想平行双目 S = T/d，程序终端打印的 raw 值，单位 m）
%   D   = 激光测距仪测得的真实距离（单位 m）
%
% 采数方法：
%   引导灯置于镖架正前方，15 ~ 30 m 每隔 0.5 m 记录一档（共 31 档），
%   并在前哨（≈17.25 m）与基地（≈26.45 m）两个工况点附近加密（±1 m 内
%   加密到 0.125~0.25 m 一档），保证工况点处拟合精度。
%   激光测距仪记录真实距离，每档记录若干次视觉 raw 值取平均（也可把全部
%   单次样本直接填入，统计上更优）。档位间距不必严格——真实距离以激光
%   读数为准，只需近似均匀覆盖并加密工况点。
%
% 拟合：二次多项式（polyfit）。
% 注意系数顺序：MATLAB polyfit 输出为降幂 [a2, a1, a0]，
%   而 configs/test.yaml 的 stereo_distance_correction_coeffs 为升幂 [a0, a1, a2]，
%   本脚本输出已按 yaml 顺序排好，直接复制即可。

clear; clc; close all

%% 1. 标定样本
% ===== 采数网格（0.5 m 基础 + 工况点加密，共 35 档） =====
%   基础网格 15:0.5:30；前哨(≈17.25)/基地(≈26.45)工况点 ±1m 内加密到 0.25 m，
%   即 [15 15.5 16 16.5 17 17.25 17.5 17.75 18 18.5 ... 26 26.25 26.5 26.75 27 27.5 28 28.5 29 29.5 30]
%   （17.5 / 26.5 已在基础网格中，加密实际新增 17.25 17.75 26.25 26.75 四档）
true_dist = [15:0.5:30, 17.25:0.25:17.75, 26.25:0.25:26.75];
true_dist = sort(unique(true_dist));                    % 35 档
fprintf('采数网格共 %d 档\n', numel(true_dist));

% TODO【占位值，采数后整体替换】：以下 35 个 raw 值按合理的非线性规律生成
%（raw 大于真实距离、差距随距离增大，与真实双目规律一致），仅供跑通脚本；
% 正式标定时替换为每档视觉 raw 实测均值（与 true_dist 逐元素对应）。
raw_mean  = [17.873 18.383 18.888 19.386 19.878 20.122 20.364 20.605 20.845 ...
             21.320 21.790 22.256 22.716 23.171 23.622 24.069 24.511 24.949 ...
             25.383 25.813 26.239 26.662 27.080 27.496 27.907 28.112 28.316 ...
             28.518 28.721 29.122 29.521 29.916 30.309 30.698 31.085];

assert(numel(raw_mean) == numel(true_dist), 'raw_mean 与 true_dist 长度必须一致');
assert(numel(raw_mean) >= 3, '二次拟合至少需要 3 个样本点');
raw_mean  = raw_mean(:);
true_dist = true_dist(:);

%% 2. 二次拟合
p = polyfit(raw_mean, true_dist, 2);   % p = [a2, a1, a0]（降幂）
a2 = p(1); a1 = p(2); a0 = p(3);

% 拟合优度 R^2
y_fit  = polyval(p, raw_mean);
ss_res = sum((true_dist - y_fit).^2);
ss_tot = sum((true_dist - mean(true_dist)).^2);
r2 = 1 - ss_res / ss_tot;

%% 3. 结果输出（可直接复制到 configs/test.yaml）
fprintf('\n===== 拟合结果 =====\n');
fprintf('D = %.6f + %.6f*raw + %.6f*raw^2\n', a0, a1, a2);
fprintf('R^2 = %.4f\n', r2);
fprintf('\n复制到 configs/test.yaml（注意顺序为 [a0, a1, a2] 升幂）：\n');
fprintf('stereo_distance_correction_coeffs: [%.6f, %.6f, %.6f]\n\n', a0, a1, a2);

%% 4. 可视化
x_plot = linspace(min(raw_mean)*0.95, max(raw_mean)*1.05, 200);

figure('Name', '双目测距二次拟合', 'Position', [100 100 1100 420]);

subplot(1, 2, 1); hold on; grid on
plot(raw_mean, true_dist, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', '标定样本');
plot(x_plot, polyval(p, x_plot), 'b-', 'LineWidth', 1.5, 'DisplayName', ...
    sprintf('D = %.4f%+.4fraw%+.4fraw^2', a0, a1, a2));
plot(x_plot, x_plot, 'k:', 'DisplayName', '理想线 D = raw');
xlabel('视觉原始距离 raw (m)'); ylabel('真实距离 D (m)');
title(sprintf('二次拟合  R^2 = %.4f', r2));
legend('Location', 'northwest');

subplot(1, 2, 2); hold on; grid on
stem(true_dist, true_dist - y_fit, 'filled');
yline(0, 'k--');
xlabel('真实距离 D (m)'); ylabel('残差 D - D_{fit} (m)');
title('拟合残差（应无随距离变化的明显趋势）');

%% 5. 残差趋势自检
% R^2 应 > 0.99；若残差随距离呈系统性趋势（单调或弯曲），说明二次骨架不足
% 或样本异常，先检查该档样本离散度，再考虑是否在代码公式侧加高阶项（需同步改 C++）。
resid = true_dist - y_fit;
fprintf('残差最大绝对值: %.3f m，RMSE: %.3f m\n', ...
    max(abs(resid)), sqrt(mean(resid.^2)));
