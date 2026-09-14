# calibration

This library calibrates one camera from chessboard images and writes plain YAML
arrays compatible with `tasks/stereo/stereo_range.cpp`.

## Build

```bash
cmake -S . -B build
cmake --build build --target calibration_tool -j
```

## Run

The default board is 11 x 8 inner corners with 25 mm squares:

```bash
./build/calibration/calibration_tool \
  --images calibration_images/left \
  --output left_calibration.yaml \
  --name left
```

For a different board, `--corners` means inner corners, not the number of
printed squares. `--square-size` is in meters:

```bash
./build/calibration/calibration_tool \
  --images calibration_images/right \
  --output right_calibration.yaml \
  --name right \
  --corners 11 8 \
  --square-size 0.025
```

The output contains keys such as:

```yaml
left_camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
left_distort_coeffs: [k1, k2, p1, p2, k3]
```

Use images captured at the same resolution, crop, orientation, and camera
settings as the deployed program. At least three views are required; 20 to 40
views covering the center and edges of the image are recommended.