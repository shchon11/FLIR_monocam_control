# flir_camera_undistort_viewer

intrinsic이 적용된 undistorted compressed 토픽을 다시 퍼블리시하는 패키지다.

## 역할

- `image_rgb/compressed` 구독
- `camera_info` 구독
- 왜곡 보정 후 `image_rgb/undistorted/compressed` 퍼블리시

## 입력과 출력

입력:

- `image_rgb/compressed`
- `camera_info`

출력:

- `image_rgb/undistorted/compressed`

namespace를 주면 모두 같이 namespace를 탄다.

예:

- `/camera0/image_rgb/compressed`
- `/camera0/camera_info`
- `/camera0/image_rgb/undistorted/compressed`

## 실행

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py namespace:=camera0
```

## 동작 방식

- compressed 이미지를 디코드
- `camera_info`의 `K`, `D`, `R`, `P`로 undistort map 생성/캐시
- 왜곡 보정 후 다시 compressed 이미지로 인코드
- 입력이 JPEG면 JPEG로, PNG면 PNG로 다시 출력

## 메모

- 기본 출력 QoS는 `best_effort`
- `rqt_image_view`에서는 `/camera0/image_rgb/undistorted/compressed` 같은 토픽을 바로 열면 된다.
- intrinsic이 아직 안 들어온 카메라에는 의미 있게 동작하지 않는다.
