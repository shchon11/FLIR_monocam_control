# FLIR Publisher

FLIR/Teledyne Spinnaker 카메라를 ROS 2 Humble에서 다루기 위한 워크스페이스입니다.

현재 포함된 핵심 패키지는 `flir_spinnaker_camera`이고, 이 노드는 다음을 퍼블리시합니다.

- `/image_raw`
- `/camera_info`
- `/image_raw/metadata`
- `/image_rgb/compressed`

핵심 목적은 이렇습니다.

- FLIR 카메라 raw 프레임을 ROS로 안정적으로 받기
- packed 10/12-bit, 16-bit, mono/Bayer 포맷을 비교하기
- camera -> PC 링크 대역폭과 ROS 내부 토픽 대역폭을 구분해서 보기
- calibration 결과를 나중에 `/camera_info`에 바로 넣을 수 있게 준비하기

## 구조

현재 파이프라인은 아래처럼 동작합니다.

1. PC가 Spinnaker로 카메라를 열고 `PixelFormat`, acquisition, stream 관련 설정을 적용합니다.
2. 카메라가 현재 설정된 포맷의 raw 프레임을 PC로 전송합니다.
3. 노드는 그 raw 버퍼를 `/image_raw`로 퍼블리시합니다.
4. 노드는 같은 프레임 기준의 메타데이터를 `/image_raw/metadata`로 퍼블리시합니다.
5. 노드는 calibration/geometry 정보를 `/camera_info`로 퍼블리시합니다.
6. 필요하면 host PC에서 RGB로 변환 후 JPEG/PNG 압축하여 `/image_rgb/compressed`를 퍼블리시합니다.

중요한 점:

- `/image_raw`는 카메라에서 받은 raw/mono/Bayer 중심 토픽입니다.
- `/camera_info`는 intrinsic, distortion, ROI, binning 같은 calibration/geometry용입니다.
- per-frame capture metadata는 `/camera_info`가 아니라 `/image_raw/metadata`로 분리했습니다.
- `/image_rgb/compressed`의 압축은 카메라 내부가 아니라 PC에서 수행됩니다.

## 요구 사항

- Ubuntu + ROS 2 Humble
- Spinnaker SDK 4.x
- Spinnaker에서 인식되는 FLIR/Teledyne 카메라
- OpenCV

참고할 Spinnaker 문서.

- `SpinView-Getting-Started.html`
- `Spinnaker C++ API Reference.html`
- `MV-Github-Examples.html`

아래 명령 예시는 워크스페이스 루트에서 실행한다고 가정합니다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select flir_spinnaker_camera
```


## 실행

기본 실행:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

시리얼 번호로 특정 카메라 선택:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py camera_serial:=12345678
```

별도 파라미터 파일 사용:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  params_file:=src/flir_spinnaker_camera/config/flir_camera.yaml
```

launch 인자 확인:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py --show-args
```

## 퍼블리시 토픽

### `/image_raw`

- 타입: `sensor_msgs/msg/Image`
- 내용: 카메라에서 받은 raw/mono/Bayer 이미지
- 주의: packed 10/12-bit로 수신한 경우에도 ROS 호환성을 위해 host에서 `16-bit`로 풀어 퍼블리시할 수 있습니다

### `/camera_info`

- 타입: `sensor_msgs/msg/CameraInfo`
- 내용: intrinsic, distortion, rectification, projection, ROI, binning
- 기본값은 비보정 상태이며, YAML로 calibration 결과를 넣도록 해두었습니다

### `/image_raw/metadata`

- 타입: `flir_spinnaker_camera/msg/FlirMetadata`
- 내용 예시:
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
- 내용: host에서 RGB8로 만든 뒤 JPEG 또는 PNG로 압축한 이미지
- 목적: ROS 내부 전송량, 저장, 원격 시각화 부담 줄이기

## 설정 파일

기본 설정 파일은 [flir_camera.yaml](src/flir_spinnaker_camera/config/flir_camera.yaml)입니다.

자주 보는 항목은 아래입니다.

- `camera_serial`
- `camera_index`
- `frame_id`
- `auto_pixel_format`
- `pixel_format`
- `publish_raw`
- `publish_camera_info`
- `publish_metadata`
- `publish_rgb_compressed`
- `rgb_compression_format`
- `rgb_jpeg_quality`
- `rgb_png_compression_level`

카메라별 writable GenICam 제어도 전부 ROS 파라미터로 노출됩니다.

- `camera.*`
- `stream.*`
- `tl_device.*`

현재 연결된 카메라에서 실제로 어떤 이름이 열리는지는 이렇게 확인하면 됩니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 param list /flir_camera
```

예를 들어 많이 쓰는 항목은 보통 이런 식입니다.

- `camera.AcquisitionFrameRateEnable`
- `camera.AcquisitionFrameRate`
- `camera.ExposureAuto`
- `camera.ExposureTime_FloatVal`
- `camera.GainAuto`
- `camera.GainDB_Val`

모델마다 이름이 조금 다를 수 있으니 실제 파라미터 목록을 기준으로 쓰는 게 안전합니다.

## Pixel Format 정리

`pixel_format`은 카메라가 PC로 어떤 프레임 포맷을 보낼지 결정합니다.

대표 예시는 이렇습니다.

- `BayerRG8`: Bayer raw 8-bit, 대략 `1 byte/pixel`
- `BayerRG12p`: Bayer raw 12-bit packed, camera -> PC 링크 절약
- `BayerRG16`: Bayer raw 16-bit container, 대략 `2 bytes/pixel`
- `Mono8`: mono 8-bit
- `Mono12p`: mono 12-bit packed
- `Mono16`: mono 16-bit
- `RGB8` 또는 `RGB8Packed`: camera가 컬러 3채널로 전송, 대략 `3 bytes/pixel`

중요한 해석:

- `BayerRG12p` 같은 packed 포맷은 camera -> PC 링크 대역폭을 줄일 수 있습니다.
- 하지만 ROS의 `sensor_msgs/Image`는 packed Bayer 10/12-bit를 표준 인코딩으로 다루기 어렵습니다.
- 그래서 이 노드는 packed raw를 host에서 `bayer_*16` 또는 `mono16`으로 풀어 `/image_raw`로 퍼블리시합니다.
- 즉 `ros2 topic bw /image_raw`만 보면 `BayerRG12p`와 `BayerRG16`이 비슷하게 보일 수 있습니다.

한 줄로 정리하면:

- 링크 대역폭 차이 확인: NIC 통계, 스위치 통계
- ROS 토픽 크기 확인: `ros2 topic bw /image_raw`

## 압축 이미지 설명

`/image_rgb/compressed`는 카메라가 압축해서 보내는 토픽이 아닙니다.

실제 흐름은 이렇습니다.

1. 카메라가 raw 또는 현재 `pixel_format` 프레임을 PC로 전송
2. host PC가 필요하면 RGB8로 변환
3. host PC가 JPEG/PNG로 압축
4. `/image_rgb/compressed` 퍼블리시

즉 압축으로 줄어드는 것은 주로 ROS 내부/원격 시각화/저장 쪽이고, camera -> PC 링크 대역폭은 그대로입니다.

압축 형식 차이:

- `jpeg`: lossy, 보통 훨씬 작고 빠름
- `png`: lossless, 보통 더 크고 CPU 사용량이 큼

압축 토픽만 쓰고 싶으면:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  publish_rgb_compressed:=true
```

압축 포맷만 바꾸고 싶으면:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  rgb_compression_format:=png \
  rgb_png_compression_level:=3
```

## CameraInfo 반영

`/camera_info`는 YAML에서 미리 calibration 값을 채워 넣도록 준비돼 있습니다.

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

- `camera_info.d: []`처럼 빈 배열은 쓰지 마세요.
- ROS 2 파라미터 파일이 빈 배열에서 `double[]` 타입을 추론하지 못합니다.
- 값이 없더라도 `0.0`들을 명시해 두는 편이 안전합니다.

## 메타데이터

per-frame 메타데이터는 `/camera_info`가 아니라 `/image_raw/metadata`로 분리했습니다.

이 토픽에는 calibration이 아니라 촬영 상태가 들어갑니다. 예를 들어:

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

토픽 타입 확인:

```bash
ros2 topic list -t | grep -E 'image_raw|camera_info|image_rgb'
```

raw 인코딩 확인:

```bash
ros2 topic echo /image_raw --once --field encoding
```

압축 포맷 확인:

```bash
ros2 topic echo /image_rgb/compressed --once --field format
```

ROS 토픽 대역폭 확인:

```bash
ros2 topic bw /image_raw
ros2 topic bw /image_rgb/compressed
```

토픽 주기 확인:

```bash
ros2 topic hz /image_raw
ros2 topic hz /image_rgb/compressed
```

카메라 파라미터 목록 확인:

```bash
ros2 param list /flir_camera
```

## 트러블슈팅

### launch 후 예전 동작이 보일 때

예전 프로세스가 살아 있으면 새 빌드가 반영되지 않습니다.

```bash
pkill -f flir_spinnaker_camera_node
```

그 뒤 다시 launch 하세요.

### 카메라가 열리지 않을 때

SpinView가 카메라를 점유 중일 수 있습니다. SpinView를 먼저 종료한 뒤 다시 실행하세요.

### `camera_info.d` 관련 에러가 날 때

YAML에서 `camera_info.d: []`를 쓰면 안 됩니다. 숫자가 들어간 배열로 적어야 합니다.

### `ros2 topic hz`가 토픽마다 다르게 보일 때

큰 raw 메시지는 CLI 구독기가 놓칠 수 있습니다. 특히 `image_raw`와 `image_rgb/compressed`를 비교할 때는 `hz` 값만으로 실제 카메라 FPS를 단정하지 말고, `bw`, NIC 통계, 카메라 설정을 같이 보는 편이 정확합니다.

## 관련 파일

- [flir_spinnaker_camera_node.cpp](src/flir_spinnaker_camera/src/flir_spinnaker_camera_node.cpp)
- [flir_camera.yaml](src/flir_spinnaker_camera/config/flir_camera.yaml)
- [flir_camera.launch.py](src/flir_spinnaker_camera/launch/flir_camera.launch.py)
- [FlirMetadata.msg](src/flir_spinnaker_camera/msg/FlirMetadata.msg)
