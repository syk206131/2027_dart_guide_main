%% fit_height.m
% 相对高度解算拟合脚本（流水线第 2 步，依赖 fit_stereo_range 的系数已填回 yaml）
%
% 【重要前提】本镖架 pitch 轴固定，高度信息只能从图像中获取：
%   用【两个相机的像素 y 偏移量的平均值】v_bar 拟合相对高度 H。
%   不采用山理工 PDF 的陀螺仪 pitch 角分解方案（D=Z·cosθ, H=Z·sinθ）——
%   我们没有 pitch 传感器，引导灯在画面中的纵向位置本身就是距离的函数，
%   给定距离 D 后，v_bar 与相对高度 H 一一对应，直接拟合即可。
%
% 模型：H = b0 + b1*D + b2*D*v_bar
%   D     = 激光测距仪实测距离（m）——用物理真值做回归自变量：几何关系
%           v̄=(H−h_c)/D 对真值严格成立，且高度系数与距离拟合解耦（重标距离后
%           高度系数仍有效）。运行时视觉 D 与真值的残差 ε 引起的 H 误差
%           ≈ ε·(H−h_c)/D，距离拟合良好时为毫米级，可忽略。
%   v_bar = 0.5*((v_l-cy_l)/fy_l + (v_r-cy_r)/fy_r)，左右相机归一化 y 偏移平均
%           （与 tasks/dart/dart_calculator.cpp 第 101-103 行完全一致）
%   H     = 引导灯相对出镖点高度零点的实际高度（m，尺测/激光测距仪）
%
% 模型由来（针孔几何推导）：
%   针孔模型 v = cy + fy*(Y/Z)，而视线与光轴夹角 α 满足 tan(α) = Y/Z，
%   故 v_bar = (v-cy)/fy = tan(α) —— 归一化像素本身就是斜率，不是角度。
%   相机水平安装（θ_c=0）时：H = h_c + D*v_bar，精确线性，无需三角函数；
%   若存在装配残余俯仰 θ_c，一阶修正为 +θ_c*D，即 b1 项。
% 系数物理意义（拟合后自检）：b0 ≈ h_c（可用尺直接量：相机光心与出镖口高差，
%   拟合前先填尺量值做参照）；b1 ≈ 0（非零即装配残余俯仰，正常 <0.01 rad 量级，
%   明显非零说明装歪或采数有误）；b2 理想值 = 1——明显偏离 1 先查样本数据。
%
% 采数流程（对应 configs/test.yaml height_coeffs 注释）：
%   固定镖架、以出镖点为高度零点，引导灯摆多档 距离×高度（覆盖前哨/基地工况），
%   每档先闭环瞄准收敛（终端 aim 计数满、|x偏差|<3px）再记录，
%   否则 roll 对 y 的耦合项未消除，H 不可信。
%   每档记录：激光实测距离 D、左右图引导灯质心 y 像素、实际高度 H。
%
% 输出顺序与 configs/test.yaml 的 height_coeffs 一致：[b0, b1, b2]（升幂，无需调序）。

clear; clc; close all

%% 0. 相机内参（从 configs/test.yaml 的 left/right_camera_matrix 复制）
% 相机矩阵第 2 行: [0, fy, cy]，即第 5、6 个元素
fy_l = 0.0;  cy_l = 0.0;   % TODO: 左相机 fy, cy
fy_r = 0.0;  cy_r = 0.0;   % TODO: 右相机 fy, cy

%% 1. 录入标定样本  ↓↓↓ 替换为你的实测数据（四向量等长） ↓↓↓
D_meas = [16.0 16.0 20.0 20.0 24.0 24.0 27.0 27.0];            % 激光实测距离（m）
v_l    = [ 300  400  300  400  300  400  300  400];             % 左图质心 y（像素）
v_r    = [ 300  400  300  400  300  400  300  400];             % 右图质心 y（像素）
H_true = [-0.5  0.5 -0.5  0.5 -0.5  0.5 -0.5  0.5];             % 实际相对高度（m）

assert(fy_l > 0 && fy_r > 0, '请先填写左右相机 fy / cy');
n = numel(D_meas);
assert(numel(v_l) == n && numel(v_r) == n && numel(H_true) == n, '四个样本向量长度必须一致');
assert(n >= 3, '三个未知数至少需要 3 个样本点（建议 8 档以上）');
D_meas = D_meas(:); v_l = v_l(:); v_r = v_r(:); H_true = H_true(:);

%% 2. 计算 v_bar 并做线性最小二乘（模型对 [b0,b1,b2] 是线性的）
v_bar = 0.5 * ((v_l - cy_l) / fy_l + (v_r - cy_r) / fy_r);
X = [ones(n, 1), D_meas, D_meas .* v_bar];
b = X \ H_true;                     % b = [b0; b1; b2]
b0 = b(1); b1 = b(2); b2 = b(3);

H_fit  = X * b;
resid  = H_true - H_fit;
ss_res = sum(resid.^2);
ss_tot = sum((H_true - mean(H_true)).^2);
r2 = 1 - ss_res / ss_tot;

%% 3. 结果输出（可直接复制到 configs/test.yaml）
fprintf('\n===== 拟合结果 =====\n');
fprintf('H = %.6f + %.6f*D + %.6f*D*v_bar\n', b0, b1, b2);
fprintf('R^2 = %.4f\n', r2);
fprintf('残差最大绝对值: %.4f m，残差标准差: %.4f m\n', max(abs(resid)), std(resid));
fprintf('\n复制到 configs/test.yaml：\n');
fprintf('height_coeffs: [%.6f, %.6f, %.6f]\n\n', b0, b1, b2);

%% 4. 可视化
figure('Name', '相对高度拟合', 'Position', [100 100 1100 420]);

subplot(1, 2, 1); hold on; grid on
plot(H_true, H_fit, 'ro', 'MarkerFaceColor', 'r');
lim = [min([H_true; H_fit]), max([H_true; H_fit])];
plot(lim, lim, 'k--');
xlabel('实际高度 H (m)'); ylabel('拟合高度 H_{fit} (m)');
title(sprintf('预测 vs 实际  R^2 = %.4f', r2));

subplot(1, 2, 2); hold on; grid on
stem(D_meas, resid, 'filled');
yline(0, 'k--');
xlabel('距离 D (m)'); ylabel('残差 H - H_{fit} (m)');
title('残差随距离分布（应无明显趋势）');

%% 5. 自检提示
% 若残差随 D 或 v_bar 呈系统性趋势：检查 ① 采数时是否每档都瞄准收敛；
% ② 高度零点是否统一为出镖点；③ 距离修正系数是否已正确填回。
