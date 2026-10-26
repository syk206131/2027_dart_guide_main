# calibration

相机标定工具集，包含三个可执行文件和一个静态库：

| 工具 | 输入 | 输出 |
|---|---|---|
| `calibration_tool` | 单相机棋盘格图片目录 | 单相机内参 + 畸变 yaml |
| `stereo_capture` | 双相机实时画面（空格触发） | 左右同步图对（BMP） |
| `stereo_calibration_tool` | 同步图对目录 + 单目内参 | R/T、baseline、相对倾角 yaml |

完整标定流程（MVS 设置、拍图要求、结果判读、接入配置）见主仓库
[README.md 的"标定"章节](../README.md#标定)。

## 编译

```bash
cmake -S .. -B ../build
cmake --build ../build --target calibration_tool stereo_capture stereo_calibration_tool -j
```

## 1. 单目标定 `calibration_tool`

```bash
../../build/calibration/calibration_tool \
  --images calibration_images/left \
  --output left_calibration.yaml \
  --name left \
  --corners 11 8 \
  --square-size 0.040
```

`--corners` 是**内角点**数，不是格子数（12×9 格的棋盘 = 11×8 内角点）；
`--square-size` 单位为米。输出：

```yaml
left_camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
left_distort_coeffs: [k1, k2, p1, p2, k3]
left_rms_error: 0.05
```

## 2. 双目同步采图 `stereo_capture`

```bash
../../build/calibration/stereo_capture \
  --config ../configs/test.yaml \
  --output calib_pairs \
  --corners 11 8
```

- 复用主程序的相机角色（`left_camera` / `right_camera`）和基于时间戳的
  `StereoFrameQueue`，保存的图对时间差不超过 `sync_max_delta_ms`
- **空格**保存当前最新的一对同步全分辨率图：`left/0001.bmp`、`right/0001.bmp`
  （目录已有图对时编号自动续接）；`q` / ESC 退出
- 画面上的棋盘格角点叠加只是取景反馈（在缩小的显示图上做），不修改保存的原图
- 建议采 25~40 对，覆盖中心/四角/四边、0.8~2.5m 多距离、多倾角，
  每对都要保证两路同时完整看到全部内角点

## 3. 双目标定 `stereo_calibration_tool`

```bash
../../build/calibration/stereo_calibration_tool \
  --left calib_pairs/left \
  --right calib_pairs/right \
  --output stereo_calibration.yaml \
  --config ../configs/test.yaml \
  --corners 11 8 \
  --square-size 0.040
```

- 左右图按文件名尾部序号配对（`0001.bmp` ↔ `0001.bmp`），数量不一致直接报错
- 默认 `CALIB_FIX_INTRINSIC`：从 `--config` 读取已标定的单目内参并固定，只解算
  R/T；加 `--refine-intrinsics` 可让内参参与联合优化
- 输出字段：

```yaml
stereo_rotation: [9 个值]           # R，满足 X_left = R * X_right + T
stereo_translation: [tx, ty, tz]    # 米
baseline_m: 0.1138                  # ||T||，填入 test.yaml
stereo_tilt_x_deg / y / z           # 两光轴相对倾角（度）
stereo_rms_error                    # 联合重投影 RMS（像素）
stereo_left_rms_error / stereo_right_rms_error
stereo_epipolar_mean_px / stereo_epipolar_max_px   # 校正后对应点行差
```

判读：倾角 <0.3°、对极行差 <0.5px 时理想平行模型可直接用；把 `baseline_m`
填入 `configs/test.yaml` 替换占位值即可。
