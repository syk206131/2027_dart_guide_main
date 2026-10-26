# 基于同济大学 sp_vision 框架的镖架制导双目测距方案

两台海康相机（8mm 长焦镜头）左右对称固定在镖架 yaw 平台上，传统视觉识别绿色圆形引导灯，
双目测距解算目标距离与高度，再结合电控下发的 roll 偏角计算**所需总动能**，
通过串口把「控制状态 + 能量」发送给电控。电控负责弹簧压缩量换算与发射。

**视觉侧不直接发像素偏移或距离**，只发三个量：`control_state`（瞄准/左调/右调）+ `energy_j`（焦耳）。
yaw 轴闭环由电控按 control_state 执行，视觉只做判定。

## 系统链路

```text
左相机线程 ─┐                          ┌─ 识别（全图粗检 + ROI 精修）
            ├─ StereoFrameQueue ─主线程─┤   双目测距 → 距离/高度修正
右相机线程 ─┘   (时间戳差 ≤ 8ms)        ├─ roll 瞄准点 + 能量解算
                                       └─ 串口发送 control_state / energy_j
```

- 两个采集线程用同一个 `steady_clock` 给帧打时间戳，主线程只配对时间差 ≤ `sync_max_delta_ms`
  的左右帧，避免运动目标视差污染
- 识别、解算、判定全部在主线程串行执行，无锁竞争

## 环境配置

理论上 sp_vision 能跑这个就能跑。依赖：

- OpenCV
- yaml-cpp
- spdlog / fmt / Eigen3 / nlohmann-json
- libusb（海康相机驱动）
- serial（串口通信）

不再依赖 OpenVINO 与任何神经网络模型。

## 编译

```bash
cmake -B build
make -C build/ -j `nproc`
```

编译产物：

| 产物 | 说明 |
|---|---|
| `build/standard` | 双目主程序（识别 + 测距 + 解算 + 串口） |
| `build/standard_single` | 单目测试程序（单相机 + 识别，调参用） |
| `build/hsv_tuner` | HSV 阈值调节工具 |
| `build/calibration/calibration_tool` | 单目标定（内参 + 畸变） |
| `build/calibration/stereo_capture` | 双目同步采图（空格保存图对） |
| `build/calibration/stereo_calibration_tool` | 双目标定（R/T、baseline、相对倾角） |
| `build/calibration/align_tool` | 相机光轴安装对准（远点对准法） |

## 运行

```bash
./build/standard
```

需要两个 USB 3.0 口分别插左右相机，串口 `/dev/ttyACM0` 连接电控。串口需要将用户加入
`dialout` 组（`sudo usermod -aG dialout $USER` 后重新登录）。

### 当前：串口通信临时关闭

电控尚未联调，`src/main.cpp` 中有 6 处「临时」标记，串口相关代码已注释、电控字段用占位：

| 位置 | 现状 | 恢复时改成 |
|---|---|---|
| `Gimbal` 构造 | 注释掉，不打开串口 | 取消注释 |
| `auto_aim_enabled` | 强制 `true` | `gimbal.mode() == io::GimbalMode::RUNNING` |
| `aim_at_outpost` | 固定 `false`（基地） | `gimbal.target() == 0` |
| `dart_calculator.compute(...)` | `roll=0`、`dart_index=0` | `gimbal.roll_rad()`、`gimbal.dart_index()` |
| 终端打印 roll / dart | 打印 0.0 / 0 | 同上 |
| `gimbal.send(send_data)` | 注释掉 | 取消注释 |

此时程序可无串口运行：识别、测距、解算、瞄准判定、画面显示全部照常，终端打印
`L/R/avg/err/D/H/E/aim` 供观察。

### 单目测试程序

```bash
./build/standard_single
```

用于调 HSV 阈值、确认左右相机对应关系（画面右下角显示相机序列号）、单路识别测试。

### HSV 调参

```bash
./build/hsv_tuner
```

滑动条实时看掩码，调好后填回 `configs/test.yaml` 的 `green_lower` / `green_upper`。

## 相机配置

### 左右相机区分

用 `serial_number` 绑定左右相机，防止 USB 枚举顺序变化导致左右颠倒：

```yaml
left_camera:
  device_index: -1
  serial_number: "DA1830826"
right_camera:
  device_index: -1
  serial_number: "DA4873204"
```

确认对应关系：先填一个序列号到 `left_camera`，跑 `standard_single` 看画面右下角 `SN:xxx`
是否匹配；不匹配说明这台是右相机，换到另一栏即可。

### 分辨率与曝光

- Width/Height = **1440×1080**，Binning/Decimation = 1，必须与标定内参一致
- `exposure_ms: 2` 手动曝光，压低环境光只保留亮灯
- `gain: 16`（dB）配合短曝光补亮度；噪点误检多时降增益

标定与运行时必须使用同一分辨率与曝光设置，否则内参失配、测距偏移。

### USB 接口

左相机 → USB 3.0 口 1，右相机 → USB 3.0 口 2。1440×1080 高帧率需要约 700MB/s，
USB 2.0 跑不满；`lsusb -t` 确认两台都在 5000M 总线上。

---

## 识别与目标选择

### 检测流程

1. 全图 HSV 分割 → 形态学开运算 → 轮廓三重筛选：面积、圆度 `4πS/P² ≥ 0.6`、
   外接矩形长宽比 `0.6 ~ 1.67`
2. **同屏多灯时先选目标**（见下），再做 ROI 精修
3. ROI 内只做一次腐蚀 + 膨胀，取最大轮廓用矩计算**亚像素质心**（`m10/m00`, `m01/m00`）

### 前哨站 / 基地选择

电控通过串口下发 `target`（**0 = 前哨站，1 = 基地**），视觉所有目标相关判断都以它为中心。

| 项目 | 前哨站（target=0） | 基地（target=1） |
|---|---|---|
| 位置 | 视野偏左 | 视野偏右 |
| 典型距离 | 17.25 m | 26.45 m |
| 同屏双灯时锁定 | 面积**大**者（更近） | 面积**小**者（更远） |
| 搜索转向 `control_state` | 1（向左） | 2（向右） |
| 距离校验基准 | `outpost_range_m` | `base_range_m` |

- 只有面积 ≥ `dual_target_area_threshold` 的灯才参与「同屏双灯」判断，避免噪点被当第二个目标
- 单灯时直接锁 `score` 最高者
- 「锁大/锁小」是相对关系，不依赖绝对像素面积，换距离后无需重调

### 双路一致性判定（优先级从高到低）

两路都锁定后，按以下顺序判定，任一命中都只发搜索指令、不进自瞄：

1. **锁了不同目标**：`|左面积 − 右面积| > stereo_area_diff_threshold`
   → 三角化无意义，发 `search_state`
2. **锁了错误目标**：测距结果更接近非目标工况点
   （`|D − 目标距离| > |D − 另一目标距离|`）→ 发 `search_state`
3. 两项都通过 → 进入正常自瞄与能量解算

单路锁定：左相机识别发 1、右相机识别发 2（粗对准到目标所在侧）。
双路都丢：朝 `search_state` 连续扫描。

> 距离校验之所以有效：前哨站与基地都在 26m 内的同一方向，两路误锁不同目标时
> 测距值会明显偏向 17.25m 或 26.45m 中的一侧。

### 与目标无关的共性

距离拟合、高度拟合、能量拟合都是距离 `D` 的函数，**两目标共用同一组系数**，
无需为前哨站单独拟合——拟合区间 15~30m 本就覆盖 17.25m 与 26.45m 两个工况点。

---

## 瞄准判定

```text
aim_error = yaw_offset − aim_point_px(roll)
```

- `yaw_offset` = 左右相机质心相对各自主点的 x 偏移平均值（像素）
- `aim_point_px` = roll 引起的系统偏差补偿量（像素），由拟合系数给出
- `|aim_error| < aim_deadband_px`（默认 3px）连续 `aim_consecutive_frames`（默认 25 帧）
  → `control_state = 0`（瞄准完成）
- 未满足时按符号给方向：`aim_error > 0` 发 2（向右），否则发 1（向左）
- 死区内但未满帧数也发方向，电控可减速精调
- 任何搜索、单路、双丢帧都会把连续计数清零

圆靶半宽：17.25m 处 3px ≈ 2.2cm，26.45m 处 3px ≈ 3.3cm，死区占半靶宽约 5%。

---

## 双目测距

去畸变后归一化视差：

```text
raw_z = baseline_m / (disparity_sign · (x_L_norm − x_R_norm))
range = a0 + a1·raw_z + a2·raw_z²
```

- `range_correction_coeffs: [a0, a1, a2]` 升幂；`[0, 1, 0]` 为理想平行光轴直通
- `baseline_m` 由双目标定给出；`disparity_sign` 符号不对时置 `-1`
- 结果超出 `min_range_m ~ max_range_m` 判为无效，本帧不解算能量
- 实测参考：17.25m 处视差约 16.6px，26.45m 处约 10.8px

---

## 串口通信

定长二进制帧，小端，结构体 `__attribute__((packed))`，CRC16 覆盖帧头到 CRC 之前的全部字节。

### 视觉 → 电控（9 字节）

```text
VC | control_state(uint8) | energy_j(float32) | crc16(uint16)
```

| 字段 | 语义 |
|---|---|
| `control_state = 0` | 瞄准完成（可发射） |
| `control_state = 1` | 向左调整 |
| `control_state = 2` | 向右调整 |
| `energy_j` | 本次发射所需**总动能**（滑台 + 镖体），单位 J |

视觉每帧都发；搜索/丢目标帧 `energy_j` 保持 0，只有两路正常锁定并完成解算才填真实值。
能量无效（测距越界）时为 0。

### 电控 → 视觉（11 字节）

```text
EC | state(uint8) | dart_index(uint8) | target(uint8) | roll_rad(float32) | crc16(uint16)
```

| 字段 | 语义 |
|---|---|
| `state` | 0x00 空闲（舱门关闭，视觉不自瞄）/ 0x01 运行（执行自瞄）/ 0x02 错误 |
| `dart_index` | 当前镖序号 0~3，用于查 `dart_masses` |
| `target` | 0 = 前哨站 / 1 = 基地 |
| `roll_rad` | roll 轴偏角（rad），**顺时针为正** |

约定：`state != 0x01` 时视觉只做识别显示，`control_state` 保持 0、不发能量。

### 底层细节

- 固定长度收发 + 帧头（`VC` / `EC`）+ CRC16 校验，校验失败计一次错误
- 连续错误超过 5000 次触发自动重连（最多 10 次，失败间隔 1s），应对赛场 USB 掉线
- 波特率 `115200`，串口读超时 20ms

---

## 拟合与解算

`tasks/dart/` 是独立的弹道解算模块，所有系数通过 yaml 加载，MATLAB 标定后填回。
**视觉只发能量，不发弹簧压缩量**——`x = sqrt(2·E_弹簧/k)` 由电控按自己的弹簧参数换算。

解算链路（每帧）：

```text
质心(u,v) → 归一化视差 → raw_z → D（距离修正）
                    ↓
        v̄ = 平均归一化 y 偏移 → H（高度解算）
                    ↓
   aim_point = c0 + c1·roll + c2·roll²  → control_state 判定
                    ↓
 E = a·m_total·(g/4)·S²/(cos²θ_eff·(S·tanθ_eff − Δh)) + b + c·roll²
```

### 1. 距离拟合 `fit_stereo_range.m`

```text
D_true = a0 + a1·raw + a2·raw²
```

- 采数：15~30m，基础 0.5m 一档；前哨（17~18m）、基地（26~27m）工况点附近加密到 0.25m
- 用激光测距实测值作为真值，视觉 `raw` 取多次平均
- 输出升幂 `[a0, a1, a2]`，直接填 `range_correction_coeffs`

### 2. 高度拟合 `fit_height.m`

```text
H = b0 + b1·D + b2·D·v̄
v̄ = 0.5·[(v_L − cy_L)/fy_L + (v_R − cy_R)/fy_R]
```

- **关键**：归一化像素 `(v−cy)/fy` 本身就是视线夹角的正切，不是角度，绝不能再套 `tan`
- 相机水平安装、θ_c≈0 时该式精确线性；`b1` 吸收装配残余俯仰，`b2` 吸收尺度误差
- 采数网格建议 17 / 20 / 23 / 26.5 / 28m × 高度 −0.5 / 0 / +0.5m 共 15 组
- 每档必须先闭环瞄准收敛再记录，否则 roll 对 y 的耦合项未消除
- 回归自变量用**激光实测 D**（物理严格，且与距离拟合解耦）
- 自检：`b0 ≈ 尺量(相机光心与出镖口高差)`、`b1 ≈ 0`、`b2 ≈ 1`

### 3. roll → yaw 瞄准点 `fit_roll_yaw.m`

```text
aim_point_px = c0 + c1·roll + c2·roll²
```

- roll 顺时针 → 弹着偏右 → 需云台向左补 → 瞄准点为正值
- 标定：调平试射定零点，再在 ±3° 内取 5 档以上，每档调 yaw 至落点回正，记录 `(roll, avg)`
- **符号约定必须与电控/MATLAB 数据一致**，否则补偿方向相反
- 自检：`c1` 量级 1.6~2.4×10³ px/rad，`c2` 远小于 `c1`
- roll 的主导误差是横向射程偏差（≈1.1m @26m / 3°roll），必须由 yaw 全权补偿

### 4. 能量拟合 `fit_energy.m`

```text
E = a·m_total·(g/4)·S² / (cos²θ_eff·(S·tanθ_eff − Δh)) + b + c·roll²
θ_eff = asin(sinθ·cos|roll|)，m_total = m_dart[dart_index] + m_sled
```

- 由 `v₀² = g·S²/(2cos²θ_eff·(S·tanθ_eff − Δh))`、`E_kin = ½·m_total·v₀²` 推出
- 对 `[a, b, c]` 是线性的，直接最小二乘求解；`a` 理想值≈1（吸收弹簧效率与比例损耗）
- `b` 固定损耗（J），`c·roll²` roll 摩擦项（J/rad²，偶函数）
- 计算时已带 `θ_eff`，roll 对弹道几何的修正是免费的二阶小量，无需额外补偿
- 采数：距离 × 高度 × roll 多档，每档调压缩量至命中装甲板，记录命中的 `(D, H, roll, dart, E_used)`
- 自检：`a` 应接近 1；逐距离分组看残差趋势，明显漂移说明 θ 设计值或滑台质量有误
- 空气阻力可用附加项 `E = (1 + k·S)·E_弹道` 吸收，需距离跨度足够否则与 `a` 共线

### 系数顺序

所有 yaml 字段都是**升幂**（`[常数, 一次, 二次]`）；MATLAB `polyfit` 输出降幂，
四个脚本都已调好顺序，直接复制即可。

---

## 标定

分四步：单目内参 → 双目 R/T 与基线 → 光轴安装对准 → 弹道系数拟合。
棋盘格：12×9 格、格宽 40mm → **内角点 11×8**（`--corners 11 8 --square-size 0.040`）。

### 第 0 步：MVS 拍图设置

- Width/Height = **1440×1080**，Binning/Decimation = 1
- Pixel Format = Bayer RG 8；手动曝光（拍棋盘格约 5ms），增益尽量低
- 保存 **BMP/PNG**，不要 JPG
- 对焦环在标定距离调清晰后，**之后不要再拧动**

### 第 1 步：左右单目标定

```bash
./build/calibration/calibration_tool \
  --images /opt/MVS/bin/Temp/DataLeft --output configs/left_calibration.yaml \
  --name left --corners 11 8 --square-size 0.040

./build/calibration/calibration_tool \
  --images /opt/MVS/bin/Temp/DataRight --output configs/right_calibration.yaml \
  --name right --corners 11 8 --square-size 0.040
```

各拍 20~40 张，覆盖中心/四角/四边、0.8~2.5m 多距离、多倾角。
合格：`detected views ≥ 20`、`RMS < 0.5px`。把内参与畸变填回 `configs/test.yaml`。

### 第 2 步：双目同步采图

```bash
./build/calibration/stereo_capture --config configs/test.yaml \
  --output calib_pairs --corners 11 8
```

空格保存一对同步全分辨率图（`left/0001.bmp` ↔ `right/0001.bmp`），采 25~40 对。
每对都要保证两路同时完整看到 11×8 角点；其中几对让棋盘格占画面 1/3 以上。

### 第 3 步：双目标定

```bash
./build/calibration/stereo_calibration_tool \
  --left calib_pairs/left --right calib_pairs/right \
  --output configs/stereo_calibration.yaml --config configs/test.yaml \
  --corners 11 8 --square-size 0.040
```

默认 `CALIB_FIX_INTRINSIC` 固定单目内参，只解 R/T。判读：`tilt < 0.3°`、
极逆行差 `< 0.5px` 时理想平行模型可直接用。把 `baseline_m` 填回 `test.yaml`。
若相对倾角过大，说明**机械安装**没调好，优先调支架而不是改算法。

### 第 3.5 步：相机光轴安装对准 `align_tool`

双目标定的 R/T 只能事后评估倾角，`align_tool` 用于**装调阶段实时对准**。原理：
无穷远点视差为 0，光轴平行时同一远点在两图像素坐标相同。

```bash
./build/calibration/align_tool --config configs/test.yaml --distance 23.5
```

- 引导灯放 **20m 以外**，用激光测距实测后传入 `--distance`（±0.3m 精度够用）
- 屏幕绿十字为本机质心，红十字为对方质心经视差修正后的投影
- **以左相机为基准调右相机支架**，把 `Δu残差` 滚动均值压到 **±0.5px** 内后锁死
- 垂直方向 `Δv` 只要几 px 内即可（对测距影响小）
- 按键：空格打印统计留档、`r` 清零重计（动过支架后按）、`q` 退出
- 0.5px 对应 raw 距离非线性 <3%，可被距离二次拟合吸收；射击振动后建议复验

### 第 4 步：接入 ballistic 系数

1. 把 `baseline_m` 填入 `configs/test.yaml`
2. 跑 `./build/standard`，灯放 2~3m 验证距离符号：合理 → `disparity_sign: 1.0`；negative/离谱 → `-1.0`
3. 按「拟合与解算」顺序依次做：**距离 → 高度 → roll 瞄准点 → 能量**（后一步依赖前一步已填回）
4. 四个 MATLAB 脚本都在 `matlab/`，输出即 yaml 字段值

---

## 参数速查（`configs/test.yaml`）

```yaml
# 识别
green_lower: [35, 100, 100]
green_upper: [85, 255, 255]
min_light_area: 5.0
max_light_area_ratio: 0.05
min_circularity: 0.6
min_aspect_ratio: 0.6
max_aspect_ratio: 1.67

# 目标决策
dual_target_area_threshold: 30.0    # 同屏双灯的第二灯面积门槛
stereo_area_diff_threshold: 100.0   # 两路面积差 → 判为不同目标
outpost_range_m: 17.25
base_range_m: 26.45

# 瞄准判定
aim_deadband_px: 3.0
aim_consecutive_frames: 25

# 测距
baseline_m: 0.12
disparity_sign: 1.0
range_correction_coeffs: [0.0, 1.0, 0.0]

# 弹道解算（占位，MATLAB 拟合后填）
height_coeffs: [0.0, 0.0, 0.0]
roll_yaw_compensation_coeffs: [0.0, 0.0, 0.0]
launch_theta_rad: 0.0
gravity: 9.7877
sled_mass_kg: 0.0
dart_masses: [0.25, 0.25, 0.25, 0.25]
energy_coeffs: [0.0, 0.0, 0.0]
```

调参建议：

- 远处灯小 → 降 `min_light_area`；误检多 → 升 `min_circularity` 或降 `gain`
- `dual_target_area_threshold` 取「低于基地 26m 处典型面积、高于噪点」
- `stereo_area_diff_threshold` 取「同目标噪声级面积差」与「前哨/基地混锁时约 2.3 倍差」之间
- 真实 yaw 偏移在 ±748px 内（理论半视场），不会与搜索状态混淆

## Tips

- MV-CS016-10UC + 8mm 镜头，17m 与 25m 处均能稳定识别引导灯
- 标定后不要用 MVS 客户端改分辨率/Binning，否则内参失配
- 镖架整体 roll 偏转（两相机相对姿态不变）不影响测距，理想平行双目模型可直接用
- 相机与出镖机构同装在一个 yaw 平台，相对位姿恒定，固定偏差由试射标定自动吸收
- 装甲板约 130×120cm，误差裕度宽：能量拟合相对误差 ≤3% 即可满足纵向命中
