# FLIR ROS Workspace

FLIR/Teledyne Spinnaker 카메라를 ROS 2 Humble에서 쓰기 위한 워크스페이스다.

이 저장소는 크게 세 가지를 맡는다.

- 카메라 스트리밍
- 체스보드 기반 캘리브레이션
- 캘리브레이션 결과를 적용한 왜곡 보정 스트림

## 패키지 맵

### `flir_spinnaker_camera`

메인 카메라 노드다.

- 실제 FLIR 카메라를 Spinnaker로 열고 프레임을 받음
- `/image_raw`, `/camera_info`, `/image_raw/metadata`, `/image_rgb/compressed` 퍼블리시
- `camera_info.yaml_path`를 통해 저장된 캘리브레이션 YAML을 `/camera_info`에 반영 가능

자세한 내용:

- [`src/flir_spinnaker_camera/README.md`](src/flir_spinnaker_camera/README.md)

### `flir_camera_calibration`

캘리브레이션 샘플 수집용 보조 패키지다.

- `/image_rgb/compressed`를 구독
- 체스보드가 보이면 `/calibration/image_annotated/compressed` 퍼블리시
- `space`로 샘플 캡처, `c`로 캘리브레이션 실행
- 결과를 `calibration/flir_camera_info.yaml`로 저장

자세한 내용:

- [`src/flir_camera_calibration/README.md`](src/flir_camera_calibration/README.md)

### `flir_camera_undistort_viewer`

캘리브레이션 결과를 적용한 왜곡 보정 스트림 퍼블리셔다.

- `/image_rgb/compressed`와 `/camera_info`를 구독
- 왜곡을 푼 결과를 `/image_rgb/undistorted/compressed`로 퍼블리시
- 출력은 기본 `best_effort`

자세한 내용:

- [`src/flir_camera_undistort_viewer/README.md`](src/flir_camera_undistort_viewer/README.md)

## 추천 워크플로

### 1. 카메라만 먼저 보기

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_spinnaker_camera
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

### 2. 캘리브레이션 수행하기

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_camera_calibration
ros2 launch flir_camera_calibration calibration.launch.py
```

OpenCV 창에서:

- `space`: 현재 보드 샘플 캡처
- `c`: 캘리브레이션 실행 및 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

### 3. 캘리브레이션 결과를 메인 카메라에 반영하기

`src/flir_spinnaker_camera/config/flir_camera.yaml`에 아래처럼 넣거나 launch 인자로 넘기면 된다.

```yaml
camera_info.yaml_path: "calibration/flir_camera_info.yaml"
```

### 4. 왜곡 보정된 스트림 퍼블리시하기

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_camera_undistort_viewer
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py
```

`rqt_image_view`에서는 `/image_rgb/undistorted/compressed`를 바로 열면 된다.

## 토픽 흐름

기본 흐름은 아래처럼 보면 된다.

1. `flir_spinnaker_camera`
2. `/image_rgb/compressed`
3. `flir_camera_calibration`
4. `calibration/flir_camera_info.yaml`
5. `flir_spinnaker_camera`의 `/camera_info`
6. `flir_camera_undistort_viewer`
7. `/image_rgb/undistorted/compressed`

## 주요 산출물

- `calibration/flir_camera_info.yaml`: 캘리브레이션 결과
- `calibration/captures/`: 캡처 샘플 이미지

`calibration/captures/`는 gitignore 처리되어 있다.

## 환경과 빌드

필수 환경:

- Ubuntu
- ROS 2 Humble
- Spinnaker SDK 4.x
- OpenCV

환경 스크립트:

```bash
source scripts/setup_flir_env.bash
```

이 스크립트는:

- ROS 2 Humble source
- `/opt/spinnaker`가 있으면 `SPINNAKER_ROOT` 설정
- `install/setup.bash` source
- `FLIR_ROS_WS` export

전체 빌드:

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install
```

## 빠른 판단 기준

- 카메라 연결/토픽 자체가 궁금하면 `flir_spinnaker_camera`
- 보드 잡히는지 보고 캘리브레이션하려면 `flir_camera_calibration`
- 캘리브레이션 결과로 왜곡 보정된 영상을 다시 쓰려면 `flir_camera_undistort_viewer`
