# FLIR Publisher

FLIR/Teledyne Spinnaker 카메라용 ROS 2 Humble 워크스페이스.

핵심 패키지:

- `flir_spinnaker_camera`
- `flir_camera_calibration`
- `flir_camera_undistort_viewer`

퍼블리시 토픽:

- `/image_raw`
- `/camera_info`
- `/image_raw/metadata`
- `/image_rgb/compressed`
- `/calibration/image_annotated/compressed`
- `/image_rgb/undistorted/compressed`

목적:

- raw 프레임 받기
- 8/12/16-bit 포맷 비교
- camera -> PC 링크 대역폭과 ROS 내부 토픽 대역폭 구분
- calibration 값을 `/camera_info`에 바로 넣기
- calibration 보드 검출/샘플 수집
- calibration 결과로 왜곡 보정된 compressed 토픽 퍼블리시

## 구조

파이프라인:

1. PC가 Spinnaker로 카메라 오픈
2. `PixelFormat`, acquisition, stream 설정 적용
3. 카메라가 raw 프레임을 PC로 전송
4. 노드가 `/image_raw` 퍼블리시
5. 노드가 `/image_raw/metadata` 퍼블리시
6. 노드가 `/camera_info` 퍼블리시
7. 필요하면 host에서 RGB로 변환 후 `/image_rgb/compressed` 퍼블리시

포인트:

- `/image_raw`: raw/mono/Bayer 이미지
- `/camera_info`: intrinsic, distortion, ROI, binning
- `/image_raw/metadata`: per-frame 촬영 메타데이터
- `/image_rgb/compressed`: host에서 만든 압축 RGB
- `/calibration/image_annotated/compressed`: calibration helper preview
- `/image_rgb/undistorted/compressed`: calibration 결과를 적용한 왜곡 보정 이미지
- 각 토픽의 `header.stamp`: 기본적으로 host 수신 시각
- 원본 장치 timestamp는 `/image_raw/metadata.camera_timestamp_ns`에 유지

## 요구 사항

- Ubuntu
- ROS 2 Humble
- Spinnaker SDK 4.x
- Spinnaker에서 인식되는 FLIR/Teledyne 카메라
- OpenCV

참고 문서:

- `SpinView-Getting-Started.html`
- `Spinnaker C++ API Reference.html`
- `MV-Github-Examples.html`

아래 명령은 워크스페이스 루트 기준.

## 외부 환경 세팅

필요한 것:

1. ROS 2 Humble 설치
2. Spinnaker SDK 설치
3. 워크스페이스 clone
4. 환경 변수 설정
5. `colcon build`

ROS:

```bash
source /opt/ros/humble/setup.bash
```

Spinnaker:

```bash
export SPINNAKER_ROOT=/path/to/spinnaker
```

기본 설치가 `/opt/spinnaker`면 그대로 둬도 됨.

helper 스크립트:

```bash
source scripts/setup_flir_env.bash
```

이 스크립트가 하는 일:

- ROS 2 Humble source
- `/opt/spinnaker`가 있으면 `SPINNAKER_ROOT` 자동 설정
- `install/setup.bash` source
- `FLIR_ROS_WS` export

처음 빌드 전이면 `install/setup.bash` 경고가 나올 수 있음. 정상.

## 빌드

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_spinnaker_camera
```

전체 빌드:

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install
```

## 실행

기본 실행:

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

시리얼 번호 지정:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py camera_serial:=12345678
```

파라미터 파일 지정:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  params_file:=src/flir_spinnaker_camera/config/flir_camera.yaml
```

launch 인자 확인:

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py --show-args
```

## 토픽

### `/image_raw`

- 타입: `sensor_msgs/msg/Image`
- 내용: raw/mono/Bayer 이미지
- packed 10/12-bit 입력도 host에서 `16-bit`로 풀어 퍼블리시할 수 있음

### `/camera_info`

- 타입: `sensor_msgs/msg/CameraInfo`
- 내용: intrinsic, distortion, rectification, projection, ROI, binning
- 기본값은 비보정 상태
- YAML로 calibration 값 반영 가능

### `/image_raw/metadata`

- 타입: `flir_spinnaker_camera/msg/FlirMetadata`
- 내용:
- pixel format
- frame id
- camera timestamp
- exposure
- gain
- gamma
- black level
- frame-rate 관련 상태

### `/image_rgb/compressed`

- 타입: `sensor_msgs/msg/CompressedImage`
- 내용: host에서 RGB8로 만든 뒤 JPEG/PNG 압축한 이미지
- 목적: ROS 내부 전송량, 저장, 원격 시각화 부담 줄이기

## 설정 파일

기본 파일:

- [flir_camera.yaml](src/flir_spinnaker_camera/config/flir_camera.yaml)

자주 보는 항목:

- `camera_serial`
- `camera_index`
- `frame_id`
- `publisher_qos_reliability`
- `publisher_qos_depth`
- `use_camera_timestamp_in_header`
- `auto_pixel_format`
- `pixel_format`
- `publish_raw`
- `publish_camera_info`
- `publish_metadata`
- `publish_rgb_compressed`
- `rgb_compression_format`
- `rgb_jpeg_quality`
- `rgb_png_compression_level`

카메라 제어 파라미터:

- `camera.*`
- `stream.*`
- `tl_device.*`

실제 이름 확인:

```bash
source scripts/setup_flir_env.bash
ros2 param list /flir_camera
```

자주 쓰는 예:

- `camera.AcquisitionFrameRateEnable`
- `camera.AcquisitionFrameRate`
- `camera.ExposureAuto`
- `camera.ExposureTime_FloatVal`
- `camera.GainAuto`
- `camera.GainDB_Val`

모델마다 이름이 조금 다를 수 있음. 실제 파라미터 목록 기준으로 보는 게 안전.

주의:

- `camera.ExposureAuto`가 `"Continuous"` 또는 `"Once"`면 `camera.ExposureTime*`은 쓰기 불가가 될 수 있음
- `camera.GainAuto`가 `"Continuous"` 또는 `"Once"`면 `camera.Gain*`도 쓰기 불가가 될 수 있음
- 수동 값을 startup override로 넣을 때는 대응하는 auto 파라미터를 `"Off"`로 두거나 수동 override를 주석 처리하는 쪽이 안전
- `use_camera_timestamp_in_header: true`는 일부 GigE/PTP 환경에서 header stamp가 비단조로 보일 수 있어 기본값은 `false`
- 기본 publisher QoS는 `reliable` + depth `20`으로 두었고, 큰 이미지 토픽에서 subscriber 쪽 드롭이 보이면 이 값을 먼저 확인하는 게 좋음

## Pixel Format

`pixel_format`은 카메라가 PC로 보내는 프레임 포맷.

대표 예:

- `BayerRG8`: Bayer raw 8-bit, 대략 `1 byte/pixel`
- `BayerRG12p`: Bayer raw 12-bit packed, camera -> PC 링크 절약
- `BayerRG16`: Bayer raw 16-bit container, 대략 `2 bytes/pixel`
- `Mono8`: mono 8-bit
- `Mono12p`: mono 12-bit packed
- `Mono16`: mono 16-bit
- `RGB8` 또는 `RGB8Packed`: 3채널 컬러, 대략 `3 bytes/pixel`

중요:

- `BayerRG12p` 같은 packed 포맷은 camera -> PC 링크 대역폭 절감용
- ROS `sensor_msgs/Image`는 packed Bayer 10/12-bit를 다루기 애매함
- 그래서 이 노드는 host에서 `bayer_*16` 또는 `mono16`으로 풀어 `/image_raw`로 퍼블리시함
- 그래서 `ros2 topic bw /image_raw`만 보면 `BayerRG12p`와 `BayerRG16`이 비슷하게 보일 수 있음

정리:

- 링크 대역폭 확인: NIC 통계, 스위치 통계
- ROS 토픽 크기 확인: `ros2 topic bw /image_raw`

## 압축 이미지

`/image_rgb/compressed`는 카메라가 압축해서 보내는 토픽이 아님.

흐름:

1. 카메라가 raw 또는 현재 `pixel_format` 프레임 전송
2. host가 필요하면 RGB8로 변환
3. host가 JPEG/PNG 압축
4. `/image_rgb/compressed` 퍼블리시

즉 압축으로 줄어드는 건 주로 ROS 내부/원격 시각화/저장 쪽. camera -> PC 링크는 그대로.

형식 차이:

- `jpeg`: lossy, 보통 작고 빠름
- `png`: lossless, 보통 더 크고 CPU 더 씀

압축 포맷 변경:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  rgb_compression_format:=png \
  rgb_png_compression_level:=3
```

## CameraInfo 반영

`/camera_info`는 YAML에서 calibration 값 반영 가능.

설정 위치:

- [flir_camera.yaml](src/flir_spinnaker_camera/config/flir_camera.yaml)

현재 반영 가능한 항목:

- `camera_info.distortion_model`
- `camera_info.d`
- `camera_info.k`
- `camera_info.r`
- `camera_info.p`
- `camera_info.binning_x`
- `camera_info.binning_y`
- `camera_info.roi.x_offset`
- `camera_info.roi.y_offset`
- `camera_info.roi.height`
- `camera_info.roi.width`
- `camera_info.roi.do_rectify`

예시:

```yaml
camera_info.distortion_model: "plumb_bob"
camera_info.d: [-0.12, 0.03, 0.0, 0.0, 0.0]
camera_info.k: [1200.0, 0.0, 960.0, 0.0, 1200.0, 540.0, 0.0, 0.0, 1.0]
camera_info.r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
camera_info.p: [1200.0, 0.0, 960.0, 0.0, 0.0, 1200.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

주의:

- `camera_info.d: []` 금지
- ROS 2 파라미터 파일이 빈 배열에서 `double[]` 타입 추론을 못함
- 값이 없으면 `0.0` 배열로 두는 편이 안전

## 메타데이터

per-frame 메타데이터는 `/camera_info`가 아니라 `/image_raw/metadata`로 분리.

들어가는 것:

- 현재 pixel format
- 카메라 frame id
- camera timestamp
- exposure auto / exposure time
- gain auto / gain
- gamma enable / gamma
- black level
- acquisition frame rate 관련 상태

확인:

```bash
ros2 topic echo /image_raw/metadata --once
```

## 자주 쓰는 확인 명령

토픽 타입:

```bash
ros2 topic list -t | grep -E 'image_raw|camera_info|image_rgb'
```

raw 인코딩:

```bash
ros2 topic echo /image_raw --once --field encoding
```

압축 포맷:

```bash
ros2 topic echo /image_rgb/compressed --once --field format
```

ROS 토픽 대역폭:

```bash
ros2 topic bw /image_raw
ros2 topic bw /image_rgb/compressed
```

토픽 주기:

```bash
ros2 topic hz /image_raw
ros2 topic hz /image_rgb/compressed
```

카메라 파라미터 목록:

```bash
ros2 param list /flir_camera
```

## 트러블슈팅

### 예전 동작이 다시 보일 때

예전 프로세스가 살아 있으면 새 빌드가 반영 안 됨.

```bash
pkill -f flir_spinnaker_camera_node
```

그 뒤 다시 launch.

### 카메라가 안 열릴 때

SpinView가 카메라 점유 중일 수 있음. 먼저 종료.

### `camera_info.d` 에러

YAML에서 `camera_info.d: []` 쓰면 안 됨. 숫자 배열로 적어야 함.

### `ros2 topic hz`가 토픽마다 다를 때

큰 raw 메시지는 CLI 구독기가 놓칠 수 있음. `hz` 값만으로 실제 카메라 FPS를 단정하지 않는 게 좋음. `bw`, NIC 통계, 카메라 설정을 같이 봐야 함.

## 관련 파일

- [flir_spinnaker_camera_node.cpp](src/flir_spinnaker_camera/src/flir_spinnaker_camera_node.cpp)
- [flir_camera.yaml](src/flir_spinnaker_camera/config/flir_camera.yaml)
- [flir_camera.launch.py](src/flir_spinnaker_camera/launch/flir_camera.launch.py)
- [FlirMetadata.msg](src/flir_spinnaker_camera/msg/FlirMetadata.msg)
- [setup_flir_env.bash](scripts/setup_flir_env.bash)
