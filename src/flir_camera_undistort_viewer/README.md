# flir_camera_undistort_viewer

ROS 2 Humble undistorted compressed-image publisher for the FLIR workspace.

입력:

- `/image_rgb/compressed`
- `/camera_info`

기능:

- compressed JPEG/PNG 디코드
- `/camera_info`의 `K`, `D`, `R`, `P`를 이용해 왜곡 보정
- 보정된 결과를 `sensor_msgs/CompressedImage` 타입의 `/image_rgb/undistorted/compressed` 로 퍼블리시
- 입력이 JPEG면 JPEG로, PNG면 PNG로 다시 인코드해서 출력
- 기본 출력 QoS는 backpressure를 줄이려고 `best_effort`
- `rqt_image_view`에서는 `/image_rgb/undistorted/compressed` 를 바로 열면 됨

## 실행

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py
```

토픽 override 예:

```bash
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py \
  input_topic:=/image_rgb/compressed \
  camera_info_topic:=/camera_info \
  output_topic:=/image_rgb/undistorted/compressed
```
