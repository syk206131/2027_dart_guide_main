%% fit_energy.m
% 能量系数拟合脚本（流水线第 4 步，依赖距离、高度拟合均已填回 yaml）
%
% 模型（视觉→电控直接发送所需总动能，滑台+镖体，单位 J）：
%   E = a * m_total * (g/4) * S^2 / (cos(th_eff)^2 * (S*tan(th_eff) - dh)) + b + c*roll^2
%   th_eff  = asin(sin(theta) * cos(|roll|))    —— roll 对弹道几何的修正
%   m_total = m_dart(dart_index) + m_sled       —— 镖+滑台总质量（kg）
%   S       = 修正后距离 D（视觉终端打印）
%   dh      = 相对高度 H（用 fit_height 填回后的视觉打印值，或实测）
%   E_used  = 命中时实际使用的能量（J）
%
% E_used 的获取（电控侧）：由弹簧压缩量换算 E = 0.5*k*x^2*效率，
%   或直接填电控档位值——此时 a 自动吸收单位换算，但电控必须与下发值同单位解释。
%   建议统一用焦耳。
%
% 模型对 [a, b, c] 是线性的：构造特征列 x1 = m_total*(g/4)*S^2/denom，x3 = roll^2，
%   最小二乘 X \ E_used 即得。theta 默认取设计值，第 6 节提供可选的 theta 联合优化。
%
% 输出顺序与 configs/test.yaml 的 energy_coeffs 一致：[a, b, c]。

clear; clc; close all

%% 0. 固定参数（与 configs/test.yaml 保持一致）
theta  = 0.0;                                   % TODO: launch_theta_rad（出射仰角设计值，rad）
g      = 9.7877;                                % gravity（当地重力加速度）
m_sled = 0.0;                                   % TODO: sled_mass_kg（滑台质量，kg，称量）
m_dart = [0.25 0.25 0.25 0.25];                 % TODO: dart_masses（逐发称量，kg）

%% 1. 录入标定样本  ↓↓↓ 替换为你的实测数据（各向量等长，每行一次命中试射） ↓↓↓
% 采数流程：变靶位（距离×高度，覆盖前哨 ~17m / 基地 ~26m 工况）× 变 roll，
%   每档调整弹簧压缩量直至命中装甲板，记录命中的那一发的下列数据。
S       = [17.0 17.0 21.0 21.0 26.0 26.0];      % 视觉打印 D（m）
dh      = [ 0.0  0.5  0.0  0.5  0.0  0.5];      % 相对高度 H（m）
roll    = [ 0.0  0.0  0.02 0.02 0.0  0.0];      % 终端打印 roll（rad）
idx     = [ 1    1    1    1    1    1   ];      % dart_index+1（第几发，1~4）
E_used  = [10.0 11.0 13.0 14.0 19.0 20.0];      % 命中能量（J）

assert(m_sled > 0, '请先填写滑台质量 m_sled');
assert(theta ~= 0, '请先填写出射仰角 theta（launch_theta_rad）');
n = numel(S);
assert(all([numel(dh), numel(roll), numel(idx), numel(E_used)] == n), '样本向量长度必须一致');
assert(n >= 3, '三个未知数至少需要 3 个样本点（建议 12 组以上）');
S = S(:); dh = dh(:); roll = roll(:); idx = idx(:); E_used = E_used(:);

%% 2. 构造特征并线性最小二乘
m_total = m_dart(idx).' + m_sled;
[~, denom0] = energy_feature(S, dh, roll, m_total, g, theta);
bad = abs(denom0) < 1e-6;
if any(bad)
    warning('%d 个样本 denom≈0（S*tan(th) 须大于 dh），已剔除', nnz(bad));
    S(bad) = []; dh(bad) = []; roll(bad) = []; E_used(bad) = []; m_total(bad) = [];
end

[x1, ~] = energy_feature(S, dh, roll, m_total, g, theta);
X = [x1, ones(numel(x1), 1), roll.^2];
coeffs = X \ E_used;                % [a; b; c]
a = coeffs(1); b = coeffs(2); c = coeffs(3);

E_fit = X * coeffs;
resid = E_used - E_fit;
r2 = 1 - sum(resid.^2) / sum((E_used - mean(E_used)).^2);

%% 3. 结果输出（可直接复制到 configs/test.yaml）
fprintf('\n===== 拟合结果 =====\n');
fprintf('E = %.6f * m_total*(g/4)*S^2/denom + %.6f + %.6f*roll^2\n', a, b, c);
fprintf('R^2 = %.4f\n', r2);
fprintf('自检：a 理想值≈1（略大，含损耗），当前 a = %.3f\n', a);
if a < 0.5 || a > 2.0
    fprintf('⚠ a 偏离 1 较多，请检查样本数据/单位（E 是否焦耳、质量是否 kg）\n');
end
fprintf('残差最大绝对值: %.3f J\n', max(abs(resid)));
fprintf('\n复制到 configs/test.yaml：\n');
fprintf('energy_coeffs: [%.6f, %.6f, %.6f]\n\n', a, b, c);

%% 4. 可视化
figure('Name', '能量系数拟合', 'Position', [100 100 1100 420]);

subplot(1, 2, 1); hold on; grid on
plot(E_used, E_fit, 'ro', 'MarkerFaceColor', 'r');
lim = [min([E_used; E_fit]), max([E_used; E_fit])];
plot(lim, lim, 'k--');
xlabel('实际命中能量 E (J)'); ylabel('拟合能量 E_{fit} (J)');
title(sprintf('预测 vs 实际  R^2 = %.4f, a = %.3f', r2, a));

subplot(1, 2, 2); hold on; grid on
stem(S, resid, 'filled');
yline(0, 'k--');
xlabel('距离 S (m)'); ylabel('残差 (J)');
title('残差随距离分布（有趋势→theta 有零偏，见第 6 节）');

%% 5. 残差趋势自检（theta 零偏排查）
% theta 取设计值若有零偏，残差会随 S 呈系统性趋势。肉眼判断：
%   按 S 排序后残差单调或弯曲 → 跑第 6 节的 theta 联合优化；
%   随机散布 → 直接用第 3 节结果填回即可。

%% 6.（可选）theta 联合优化：仅当上一步残差随 S 有明显趋势时启用
% 用 fminsearch（基础 MATLAB，无需工具箱）在 theta 上做一维搜索，
% 每个 theta 内层仍线性解 [a,b,c]，取残差平方和最小者。
do_fit_theta = false;
if do_fit_theta
    obj = @(th) theta_obj(th, S, dh, roll, m_total, g, E_used);
    [theta_opt, sse_opt] = fminsearch(obj, theta);

    [x1o, ~] = energy_feature(S, dh, roll, m_total, g, theta_opt);
    Xo = [x1o, ones(numel(S), 1), roll.^2];
    co = Xo \ E_used;
    fprintf('\n===== theta 联合优化结果 =====\n');
    fprintf('theta: %.6f → %.6f rad（%.3f°），SSE: %.4f → %.4f\n', ...
        theta, theta_opt, rad2deg(theta_opt), sum(resid.^2), sse_opt);
    fprintf('energy_coeffs: [%.6f, %.6f, %.6f]\n', co(1), co(2), co(3));
    fprintf('launch_theta_rad: %.6f\n', theta_opt);
    fprintf('仅当 theta_opt 物理上合理（接近机械设计值）且 R^2 明显提升时才采用。\n');
end

%% ── 局部函数 ──
function [x1, denom] = energy_feature(S, dh, roll, m_total, g, theta)
% 特征列 x1 = m_total*(g/4)*S^2 / (cos(th_eff)^2 * (S*tan(th_eff) - dh))
    sin_eff = sin(theta) .* cos(abs(roll));
    sin_eff = max(-1, min(1, sin_eff));           % 数值保护
    th_eff  = asin(sin_eff);
    denom   = cos(th_eff).^2 .* (S .* tan(th_eff) - dh);
    x1      = m_total .* (g * 0.25) .* S.^2 ./ denom;
    x1(abs(denom) < 1e-6) = 0;
end

function sse = theta_obj(theta, S, dh, roll, m_total, g, E_used)
% 给定 theta，内层线性解 [a,b,c]，返回残差平方和
    [x1, denom] = energy_feature(S, dh, roll, m_total, g, theta);
    if any(abs(denom) < 1e-6), sse = inf; return; end
    X = [x1, ones(numel(x1), 1), roll.^2];
    co = X \ E_used;
    r = E_used - X * co;
    sse = sum(r.^2);
end
