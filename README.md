# 基于同济大学 sp_vision 框架的镖架制导双目测距方案

使用两个海康相机（配 8mm 长焦镜头）采集画面，传统视觉识别绿色圆形引导灯，双目测距，向电控发送像素偏移量和距离。

## 环境配置

理论上 sp_vision 能跑这个就能跑。

依赖：
- OpenCV
- yaml-cpp
- spdlog
- libusb（海康相机驱动）
- serial（串口通信）

## 编译

```bash
cmake -B build
make -C build/ -j `nproc`
```

编译产物：
- `build/standard` — 双目主程序（两相机 + 识别 + 测距 + 串口通信）
- `build/standard_single` — 单目测试程序（单相机 + 识别，用于调参和确认左右相机）
- `build/hsv_tuner` — HSV 阈值调节工具
- `build/calibration/calibration_tool` — 单目标定工具（分别标定左右相机内参）
- `build/calibration/stereo_capture` — 双目同步采图工具（空格保存同步图对）
- `build/calibration/stereo_calibration_tool` — 双目标定工具（求 R/T 与基线 baseline）

## 运行

### 双目主程序

```bash
./build/standard
```

需要两个 USB 3.0 口分别插左右相机，串口 `/dev/ttyACM0` 连接电控。

串口需要将用户加入 `dialout` 组（`sudo usermod -aG dialout $USER` 后重新登录）。无串口设备时程序会报 `Failed to open serial`，此时可在 `src/main.cpp` 中搜索 `[SERIAL-DISABLED]` 注释掉三段串口代码（include / Gimbal 构造 / send）做纯视觉调试。

### 单目测试程序

```bash
./build/standard_single
```

用于：
- 调 HSV 阈值（配合 `hsv_tuner`）
- 确认左右相机对应关系（画面右下角显示当前相机序列号）
- 单路识别测试

### HSV 调参工具

```bash
./build/hsv_tuner
```

带滑动条实时调 HSV 阈值，看掩码效果，调好后把值填到 `configs/test.yaml` 的 `green_lower` / `green_upper`。

## 相机配置

### 左右相机区分

`configs/test.yaml` 中用 `serial_number` 绑定左右相机，防止 USB 枚举顺序变化导致左右颠倒：

```yaml
left_camera:
  device_index: -1
  serial_number: "DA1830826"   # 左相机序列号
  user_defined_name: ""
right_camera:
  device_index: -1
  serial_number: "DA4873204"   # 右相机序列号
  user_defined_name: ""
```

**确认左右对应关系**：
1. 先填一个序列号到 `left_camera.serial_number`
2. 跑 `./build/standard_single`
3. 看画面右下角 `SN:xxx` 是否是你填的序列号
   - 是 → 当前这台就是左相机
   - 否 → 这台是右相机，把序列号填到 `right_camera` 下

### USB 接口安排

- 左相机 → USB 3.0 口 1
- 右相机 → USB 3.0 口 2
- USB 2.0 口 → 键盘/鼠标

1440×1080 @ 150fps 需要约 700MB/s 带宽，USB 2.0（40MB/s）跑不满，必须 USB 3.0。

验证：`lsusb -t` 确认两台相机都在 5000M 总线下。

## 双目测距原理

`standard` 由两个相机采集线程和主线程组成：

- 左右相机线程使用同一个 `steady_clock` 时间基准给帧打时间戳，写入 `StereoFrameQueue`
- 主线程只取时间差不超过 `sync_max_delta_ms`（默认 8ms）的左右帧进行配对、识别、测距

检测流程：
1. 全图 HSV 分割 → 形态学清理 → 轮廓三重筛选（面积/圆度/长宽比）
2. 双目标选择：单路画面出现两个灯时锁定面积更小者（远处=基地）
3. ROI 内二次精修质心，提高亚像素精度
4. 两路决策：面积差判别是否锁定同一目标，双目测距校验目标身份（前哨站/基地）

测距公式（去畸变后归一化视差）：

```text
raw_z = baseline_m / (x_left_normalized - x_right_normalized)
range = a0 + a1 * raw_z + a2 * raw_z^2
```

`range_correction_coeffs` 顺序是 `[a0, a1, a2]`，当前 `[0, 1, 0]` 是理想平行光轴模型。`baseline_m`、左右内参、畸变参数和拟合系数需要实测标定后填入。如果左右图像视差符号相反，将 `disparity_sign` 改为 `-1`。

## 识别算法参数

`configs/test.yaml` 中：

```yaml
green_lower: [35, 100, 100]      # HSV 下限 [H, S, V]
green_upper: [85, 255, 255]      # HSV 上限
min_light_area: 30.0             # 最小轮廓面积 (px^2)
max_light_area_ratio: 0.05       # 最大轮廓面积占整帧比例
min_circularity: 0.6             # 最小圆度 4*pi*S/P^2，正圆为 1
min_aspect_ratio: 0.6            # 外接矩形宽/高最小值
max_aspect_ratio: 1.67           # 外接矩形宽/高最大值

# 双目标决策（前哨站在左 ~17.25m，基地在右 ~26.45m，目标是基地）
dual_target_area_threshold: 30.0   # 判定"前哨站+基地同框"的面积门槛 (px^2)
stereo_area_diff_threshold: 100.0  # 两路面积差超过此值判为锁定不同目标 (px^2)
outpost_range_m: 17.25             # 前哨站距离，距离校验用
base_range_m: 26.45                # 基地距离，距离校验用
```

调参建议：
- 先用 `hsv_tuner` 调 `green_lower` / `green_upper`
- 远处灯小 → 降 `min_light_area`
- 误检多 → 升 `min_circularity`
- 灯被压扁 → 放宽 `min/max_aspect_ratio`
- `dual_target_area_threshold`：低于基地灯在 26m 处的典型面积、高于噪点，保证两个灯都能参与同框判断
- `stereo_area_diff_threshold`：同目标时两路面积差只有噪声级（几个 px²），前哨站/基地混锁时差约 2.3 倍，阈值设在两者之间；终端打印的实际面积可用于标定该值

## 串口通信

发往电控的 `VC` 帧格式（13 字节 packed）：

```text
VC | status(uint8) | yaw_offset(float) | distance_m(float) | crc16(uint16)
```

- `status` bit 0：两路均锁定基地（同一目标且距离校验通过）；bit 1：测距有效
- `yaw_offset`：左右偏移量平均值（像素）；±1000 为搜索方向哨兵
- `distance_m`：距离（米），无效值 -1；锁定前哨站时仍填真实测量值

发送逻辑（目标是基地；前哨站在左 ~17.25m，基地在右 ~26.45m，两者夹角很小）：

| 识别情况 | status | yaw_offset | distance_m |
|---|---|---|---|
| 两路都没识别 | 0x00 | 0.0 | -1.0 |
| 仅左相机识别 | 0x00 | -1000.0（哨兵，提示云台向左搜） | -1.0 |
| 仅右相机识别 | 0x00 | +1000.0（哨兵，提示云台向右搜） | -1.0 |
| 两路锁定不同目标（面积差 > `stereo_area_diff_threshold`） | 0x00 | +1000.0（左锁前哨站/右锁基地，向右搜） | -1.0 |
| 两路锁定前哨站（测距更接近 `outpost_range_m`） | 0x00 | +1000.0（向右搜基地） | 真实测量距离（≈17.25m，供电控复核） |
| 两路锁定基地 | 0x01 | 两路偏移平均值 | 测距有效时置 bit 1 并填真实距离 |

目标选择规则：

- 单相机视野同时出现两个灯（前哨站+基地同框）时，锁定**面积更小**的那个（基地更远，看起来更小）；只有面积超过 `dual_target_area_threshold` 的灯才参与"双目标"判断，防止噪点被当成第二个目标
- 两路都锁定后比较面积：同一目标在两路画面中大小几乎一致；面积差大说明左相机锁了前哨站、右相机锁了基地，此时三角化无意义，发哨兵向右搜直到两路都锁定基地
- 面积相近时用双目测距校验目标身份：距离更接近 17.25m（`outpost_range_m`）说明两路锁的是前哨站，发哨兵向右搜基地；距离字段仍填真实测量值，电控可用同样判据复核
- 正常跟踪真实偏移量在 ±748px 以内（理论值），±1000 哨兵不会与真实值混淆

## 标定

`calibration/` 目录有三个标定工具，标定分两步：**先分别做左右单目标定（内参），再做双目标定（R/T、基线）**。

棋盘格规格：12×9 个格子、格宽 40mm → **内角点 11×8**（内角点 = 格子数 − 1，命令里填 `--corners 11 8 --square-size 0.040`，容易填错）。

### 第 0 步：MVS 拍图设置

用海康 MVS 客户端拍图时（标定后不要再改这些设置）：

- Width/Height = **1440×1080**，Binning/Decimation = 1（与运行分辨率一致）
- Pixel Format = Bayer RG 8；手动曝光（拍棋盘格 5ms 左右，白格不过曝），Gain 尽量低
- 保存 **BMP/PNG**，不要 JPG
- 对焦环在标定距离（1.5~2m）调清晰后，**之后不要再拧动**

### 第一步：左右相机分别单目标定（求内参）

分别连接两台相机，各拍 20~40 张（覆盖画面中心/四角/四边、多距离、多倾角，11×8 角点完整可见），图片分目录存放：

```bash
./build/calibration/calibration_tool \
  --images /opt/MVS/bin/Temp/DataLeft \
  --output configs/left_calibration.yaml \
  --name left --corners 11 8 --square-size 0.040

./build/calibration/calibration_tool \
  --images /opt/MVS/bin/Temp/DataRight \
  --output configs/right_calibration.yaml \
  --name right --corners 11 8 --square-size 0.040
```

合格标准：`detected views` ≥ 20，`RMS error < 0.5 px`（越小越好）。把输出的
`left/right_camera_matrix`、`left/right_distort_coeffs` 填入 `configs/test.yaml`
（项目已完成此步，见 yaml 内注释）。

### 第二步：双目同步采图（为求 R/T、基线）

双目标定要求每对图是**左右相机在同一时刻看到同一块棋盘格**，不能用 MVS 分别拍的单目图配对。用同步采图工具（复用主程序的双相机采集 + 时间戳同步队列）：

```bash
# 拍棋盘格时画面较暗，可临时把 test.yaml 的 exposure_ms 改为 5，采完改回 2
./build/calibration/stereo_capture \
  --config configs/test.yaml \
  --output calib_pairs \
  --corners 11 8
```

- 左右画面并排显示，绿色角点叠加表示两路都检出棋盘格
- 棋盘格拿稳静止后按**空格**保存当前同步图对（全分辨率 BMP，序号严格对应）
- 采 **25~40 对**，`q` 退出；图保存在 `calib_pairs/left/` 和 `calib_pairs/right/`
- 摆位：画面中心/四角/四边、0.8~2.5m 多距离、各方向倾斜 15°~30°，**每对都要保证两路同时完整看到 11×8 角点**；其中几对让棋盘格占画面 1/3 以上（决定基线尺度精度）

### 第三步：双目标定（输出 R/T、baseline、相对倾角）

```bash
./build/calibration/stereo_calibration_tool \
  --left calib_pairs/left \
  --right calib_pairs/right \
  --output configs/stereo_calibration.yaml \
  --config configs/test.yaml \
  --corners 11 8 --square-size 0.040
```

默认 `CALIB_FIX_INTRINSIC`：固定第一步的优秀单目内参，只解算 R/T（推荐；图对数量少于单目视图，重估内参反而可能变差）。加 `--refine-intrinsics` 可让左右内参参与联合优化。

输出示例与判读：

```text
detected pairs: 32/34              # 两路同时检出角点的图对，至少需要 5
joint RMS: 0.04 px                 # 联合重投影误差，<0.5 合格
baseline:  113.85 mm (0.11385 m)   # ← 把这个值填到 test.yaml 的 baseline_m
T (m):     [0.1138, -0.0011, ...]  # T.x 应占主导；T.y/T.z 过大说明安装错位
tilt (deg): x=0.05 y=-0.02 z=0.10  # 两光轴相对倾角，<0.3° 可继续用理想平行模型
epipolar row error: mean=0.03 px   # 校正后对应点行差，越小越好
```

### 第四步：把结果接入运行配置

1. 把输出的 `baseline_m` 填入 `configs/test.yaml`（替换占位 0.12）
2. 跑 `./build/standard`，灯放 2~3m 验证：距离合理为正 → `disparity_sign: 1.0`；距离 invalid/为负/离谱 → 改为 `-1.0`
3. 在 1.5/3/5m 实测距离，残差呈系统性规律时拟合 `range_correction_coeffs: [a0,a1,a2]`
4. 若相对倾角 >0.3° 且需要远距离高精度：需要接入 `stereoRectify` 立体校正（检测前 remap），这是测距链路的后续扩展

## Tips

- 经测试，MV-CS016-10UC 相机 + 8mm 镜头，在 17 米和 25 米处均能稳定准确识别引导灯
- 曝光 `exposure_ms: 2` 手动设置，压环境光，只留亮灯
- 标定后不要用 MVS 客户端动相机分辨率/ Binning，否则内参失配
