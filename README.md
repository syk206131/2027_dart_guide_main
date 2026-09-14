# 基于同济大学sp_vision框架和和深圳大学开源模型的镖架制导
## 环境配置
理论上sp_vision能跑这个就能跑
##  编译
    ```bash
    
    ```cmake -B build
    make -C build/ -j `nproc`

## 运行
    ```bash./build/standard
    
    
    ```

## 双目测距版前端

`standard` 现在由两个相机采集线程和主线程组成：左右相机线程使用同一个
`steady_clock` 时间基准给帧打时间戳并写入 `StereoFrameQueue`，主线程只取时间差
不超过 `sync_max_delta_ms` 的左右帧进行检测、自瞄和测距。检测框会先从网络输入
尺寸还原到原始相机像素坐标，再用 HSV 绿色区域质心精修引导灯中心。

在 `configs/test.yaml` 中，`left_camera.device_index` 和
`right_camera.device_index` 是按 `vid_pid` 筛选后的零基设备序号。实机建议改填
两个相机的 `serial_number`，这样 USB 枚举顺序变化时左右相机不会交换。

测距使用去畸变后的归一化视差：

```text
raw_z = baseline_m / (x_left_normalized - x_right_normalized)
range = a0 + a1 * raw_z + a2 * raw_z^2
```

其中 `range_correction_coeffs` 的顺序是 `[a0, a1, a2]`。当前 `[0, 1, 0]` 只是
理想平行光轴模型，`baseline_m`、左右内参、畸变参数和拟合系数需要换成实测值。
如果左右图像中的视差符号相反，将 `disparity_sign` 改为 `-1`。

发往电控的 `VC` 帧已扩展为 18 字节（小端机器上的 packed 布局）：

```text
VC | status(uint8) | reserved(uint8) | yaw_offset(float)
   | yaw_abs_offset_difference(float) | distance_m(float) | crc16(uint16)
```

`yaw_offset` 是左右偏移量的有符号平均值，用于保留原来的 yaw 自瞄；
`yaw_abs_offset_difference = abs(abs(left_offset) - abs(right_offset))`，由电控
据此自行判断“连续 5 帧 <= 3 px”。`status` bit 0 表示左右均检测到引导灯，bit 1
表示测距有效；无效值为 `-1`。`distance_m` 单位为米。


## 串口通信
TBD

## tips:
经测试，MV-CS016-10UC相机+8mm镜头 ，即自瞄常用的配置，在17米处和25米处均能稳定准确识别引导灯。
