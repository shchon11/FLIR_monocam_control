# FLIR ROS Workspace

FLIR/Teledyne Spinnaker 카메라를 ROS 2 Humble에서 운용하기 위한 워크스페이스다.

이 저장소는 이제 아래 4개 층으로 관리한다.

- logical camera name: `camera0`, `camera1`
- device inventory: 어떤 logical camera가 어떤 serial인지
- intrinsics: 각 물리 카메라의 내부 파라미터
- extrinsics: rig 안에서 logical camera frame이 어디 붙는지

## 현재 관리 구조

### 1. 공통 카메라 파라미터

- `src/flir_spinnaker_camera/config/flir_camera.yaml`

모든 카메라가 공유하는 기본 제어 파라미터를 둔다.

예:

- pixel format
- publish/QoS 기본값
- camera / stream / tl_device 공통 설정

### 2. 카메라 inventory

- `src/flir_spinnaker_camera/config/flir_cameras.yaml`

logical name과 physical serial, frame_id, intrinsic 파일 경로를 연결한다.

예:

- `camera0 -> serial 24293695`
- `camera1 -> serial 24401042`

### 3. Intrinsics

- `calibration/intrinsics/<serial>.yaml`

intrinsic calibration은 physical camera + lens 조합에 묶이므로 serial 기준으로 저장한다.

`calibration/intrinsics/*.yaml`는 로컬 calibration artifact로 보고 gitignore 처리한다.

현재 예:

- `calibration/intrinsics/24293695.yaml`

### 4. Extrinsics

- `calibration/extrinsics/rig.yaml`

extrinsic은 logical camera name 기준으로 관리한다.

예:

- `camera0_optical_frame`
- `camera1_optical_frame`
- parent `camera_rig`

## 패키지 맵

### `flir_spinnaker_camera`

메인 카메라 드라이버다.

- physical camera를 Spinnaker로 열고 프레임을 받음
- `/image_raw`, `/camera_info`, `/image_raw/metadata`, `/image_rgb/compressed` 퍼블리시
- inventory에서 지정한 intrinsic YAML을 `/camera_info`에 반영

자세한 내용:

- [`src/flir_spinnaker_camera/README.md`](src/flir_spinnaker_camera/README.md)

### `flir_camera_calibration`

체스보드 기반 intrinsic calibration helper다.

- `/image_rgb/compressed` 구독
- `/calibration/image_annotated/compressed` 퍼블리시
- calibration 결과를 `calibration/intrinsics/<serial>.yaml`로 저장

자세한 내용:

- [`src/flir_camera_calibration/README.md`](src/flir_camera_calibration/README.md)

### `flir_camera_undistort_viewer`

intrinsic이 적용된 undistorted compressed 토픽 퍼블리셔다.

- `/image_rgb/compressed`와 `/camera_info` 구독
- `/image_rgb/undistorted/compressed` 퍼블리시

자세한 내용:

- [`src/flir_camera_undistort_viewer/README.md`](src/flir_camera_undistort_viewer/README.md)

## 추천 워크플로

### 1. inventory 편집

`src/flir_spinnaker_camera/config/flir_cameras.yaml`에서 카메라 목록을 관리한다.

### 2. 멀티카메라 launch

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_spinnaker_camera flir_multi_camera.launch.py
```

기본 토픽 예:

- `/camera0/image_rgb/compressed`
- `/camera0/camera_info`
- `/camera1/image_rgb/compressed`
- `/camera1/camera_info`

### 3. camera0 intrinsic calibration

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_camera_calibration calibration.launch.py namespace:=camera0
```

저장 결과:

- `calibration/intrinsics/<serial>.yaml`

### 4. camera0 undistort stream

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py namespace:=camera0
```

출력:

- `/camera0/image_rgb/undistorted/compressed`

## 실제 검증한 것

현재 두 대 카메라가 연결된 상태에서 아래를 확인했다.

- inventory 기반 멀티 launch로 `camera0`, `camera1` 두 노드 기동
- `camera0`에 intrinsic YAML 자동 로드
- `camera_rig -> camera0_optical_frame`, `camera_rig -> camera1_optical_frame` static TF 기동
- `flir_camera_undistort_viewer`를 `namespace:=camera0`로 띄웠을 때 `/camera0/image_rgb/undistorted/compressed` 생성
- `flir_camera_calibration`를 `namespace:=camera0`로 띄웠을 때 `/camera0/calibration/image_annotated/compressed` 생성

## 메모

- `camera0`, `camera1`는 logical name이다.
- intrinsic은 serial 기준, extrinsic은 logical name 기준으로 관리한다.
- 지금처럼 두 카메라를 한 링크에 붙이면 공통 파라미터가 실제로 둘 다에 유효한지 별도 확인이 필요하다.
- `calibration/captures/`는 gitignore 처리되어 있다.
- `calibration/intrinsics/*.yaml`도 gitignore 처리되어 있다.
