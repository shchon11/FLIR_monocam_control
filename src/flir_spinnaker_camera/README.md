# flir_spinnaker_camera

FLIR/Spinnaker 카메라를 ROS 2 토픽으로 내보내는 메인 패키지다.

## 역할

- 카메라 오픈 및 acquisition
- raw 이미지 퍼블리시
- CameraInfo 퍼블리시
- per-frame metadata 퍼블리시
- RGB compressed 이미지 퍼블리시

## 토픽

- `image_raw`
- `camera_info`
- `image_raw/metadata`
- `image_rgb/compressed`

namespace를 주면 토픽도 같이 들어간다.

예:

- `/camera0/image_rgb/compressed`
- `/camera1/camera_info`

## 설정 파일

### 공통 카메라 파라미터

- `config/flir_camera.yaml`

여기에는 모든 카메라가 공유할 기본값을 둔다.

예:

- `pixel_format`
- publish/QoS 기본값
- 공통 `camera.*`, `stream.*`, `tl_device.*`

### 카메라 inventory

- `config/flir_cameras.yaml`

여기에는 logical camera name별로 아래를 둔다.

- `namespace`
- `serial`
- `frame_id`
- `intrinsic_file`

필요하면:

- `parameter_overrides`

## launch

단일 카메라:

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

시스템 전체:

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_spinnaker_camera flir_multi_camera.launch.py
```

`flir_multi_camera.launch.py`는:

- `flir_cameras.yaml`을 읽음
- 공통 `flir_camera.yaml`을 각 카메라 노드에 복제 적용
- inventory 기준으로 `camera0`, `camera1` 노드 생성
- `calibration/extrinsics/rig.yaml`을 읽어 static TF 퍼블리셔 생성

## CameraInfo / intrinsic 연결

각 카메라 노드는 inventory에 적힌 `intrinsic_file`을 `camera_info.yaml_path`로 받아 `/camera_info` 퍼블리시에 반영한다.

예:

- `camera0 -> calibration/intrinsics/24293695.yaml`

## 메모

- `selected_camera_serial` 파라미터는 실제 선택된 카메라 serial을 노출한다.
- calibration helper는 이 값을 읽어서 `<serial>.yaml` 파일명으로 저장한다.
- 멀티카메라에서는 공통 파라미터가 정말 둘 다에 유효한지 실제 링크 예산까지 같이 봐야 한다.
