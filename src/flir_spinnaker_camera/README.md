# flir_spinnaker_camera

FLIR 카메라에서 한 프레임을 받아

- 원본(raw) 이미지는 `image_raw`
- RGB 변환 이미지는 `image_rgb`
- 압축 RGB 이미지는 `image_rgb/compressed`

로 퍼블리시하는 ROS 2 Humble 패키지입니다.

구현은 로컬에 설치된 Spinnaker 문서와 예제를 기준으로 구성했습니다.

- `/usr/share/doc/spinnaker-doc/SpinView-Getting-Started.html`
- `/usr/share/doc/spinnaker-doc/Spinnaker C++ API Reference.html`
- `/opt/spinnaker/src/Acquisition/Acquisition.cpp`
- `/opt/spinnaker/src/ImageFormatControl/ImageFormatControl.cpp`

## 요구 사항

- ROS 2 Humble
- Spinnaker SDK 4.x
- FLIR/Teledyne 카메라가 Spinnaker에서 인식되는 상태

## 빌드

```bash
cd /home/shchon11/projects/DM/Flir_ROS_Bandwidth_test
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
```

## 실행

```bash
cd /home/shchon11/projects/DM/Flir_ROS_Bandwidth_test
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

특정 카메라를 시리얼로 선택하려면:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py camera_serial:=12345678
```

## 퍼블리시 토픽

- `/image_raw`
- `/image_rgb`
- `/image_rgb/compressed`

## 주요 파라미터

- `camera_serial`: 비우면 `camera_index` 기준으로 선택
- `camera_index`: 기본값 `0`
- `frame_id`: 기본값 `flir_camera_optical_frame`
- `auto_pixel_format`: ROS에서 다루기 쉬운 Bayer/Mono 포맷으로 자동 설정
- `pixel_format`: 예) `BayerRG8`, `BayerRG12p`, `BayerRG16`, `Mono10p`, `RGB8`, `RGB8Packed`
- `buffer_handling_mode`: 기본값 `NewestOnly`
- `color_processing`: 기본값 `hq_linear`
- `publish_rgb_compressed`: 기본값 `true`
- `rgb_compression_format`: `jpeg` 또는 `png`
- `rgb_jpeg_quality`: JPEG 품질, 기본값 `90`
- `rgb_png_compression_level`: PNG 압축 레벨, 기본값 `3`

## 픽셀 포맷

`pixel_format`은 카메라가 PC로 어떤 형태의 프레임을 보낼지 정하는 설정입니다.

- `BayerRG8`: Bayer raw 8-bit, 대략 `1 byte/pixel`
- `BayerRG16`: Bayer raw 16-bit container, 대략 `2 bytes/pixel`
- `BayerRG12p`: Bayer raw 12-bit packed, camera->PC 링크 절약
- `Mono8` / `Mono16`: 흑백 raw
- `RGB8` / `RGB8Packed` / `BGR8`: 카메라가 이미 3채널 컬러로 전송, 대략 `3 bytes/pixel`

일부 카메라는 `RGB8` 대신 `RGB8Packed`라는 enum 이름만 지원합니다. 이 노드는 `pixel_format: "RGB8"`를 주면 가능한 경우 `RGB8Packed`로 자동 해석합니다.

이 카메라에서 확인한 대표 지원 포맷은 `BayerRG8`, `BayerRG16`, `BayerRG10p`, `BayerRG10Packed`, `BayerRG12p`, `BayerRG12Packed`, `Mono8`, `Mono16`, `Mono10p`, `Mono10Packed`, `Mono12p`, `Mono12Packed`입니다.

중요한 점:

- packed 10/12-bit 포맷은 `camera -> PC` 링크 대역폭을 줄이는 데 유리합니다.
- 하지만 ROS `sensor_msgs/Image`는 표준 Bayer 10/12 인코딩 문자열이 없어서, 이 노드는 host에서 이를 `16-bit`로 풀어 `/image_raw`를 `bayer_*16` 또는 `mono16`으로 퍼블리시합니다.
- 그래서 packed 포맷의 이득은 주로 카메라 링크에서 생기고, `ros2 topic bw /image_raw`에서는 그대로 안 보일 수 있습니다.

## RGB 압축

`image_rgb/compressed`는 카메라가 압축해서 보내는 게 아니라, PC가 받은 프레임을 host 쪽에서 압축해서 퍼블리시하는 토픽입니다.

파이프라인은 이렇습니다.

- 카메라가 raw 또는 현재 `pixel_format` 프레임을 PC로 전송
- 필요하면 PC가 `RGB8`로 변환해서 `/image_rgb` 생성
- 같은 `RGB8` 프레임을 OpenCV로 JPEG/PNG 압축해서 `/image_rgb/compressed` 생성

즉 압축이 줄여주는 것은 `PC 내부/ROS 토픽/시각화` 쪽 대역폭이지, `camera -> PC` 링크 대역폭은 아닙니다.

압축 형식 차이:

- `jpeg`: lossy, 보통 훨씬 작고 빠름
- `png`: lossless, 보통 더 크고 CPU 사용량이 큼

실제로 압축만 쓰고 싶으면 uncompressed RGB를 끄면 됩니다.

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  publish_rgb:=false \
  publish_rgb_compressed:=true \
  rgb_compression_format:=jpeg \
  rgb_jpeg_quality:=90
```

## 카메라 제어 파라미터

이제 카메라의 writable GenICam 노드가 전부 ROS 파라미터로 열립니다.

- `camera.*`: 카메라 노드맵
- `stream.*`: TL stream 노드맵
- `tl_device.*`: TL device 노드맵

현재 연결된 카메라에서 실제로 쓸 수 있는 이름은 이렇게 확인하면 됩니다.

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 param list /flir_camera
```

이 `BFS-PGE-27S5C`에서는 프레임레이트/노출/게인이 이런 이름으로 보였습니다.

- `camera.AcquisitionFrameRateEnable`
- `camera.AcquisitionFrameRate`
- `camera.ExposureAuto`
- `camera.ExposureTime_FloatVal`
- `camera.GainAuto`
- `camera.GainDB_Val`

일부 모델은 프레임레이트 값 노드가 `camera.FrameRateHz_Val`처럼 다른 이름으로 보일 수 있으니, 실제 이름은 `ros2 param list /flir_camera` 결과를 기준으로 쓰면 됩니다.

런타임 제어 예시:

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 param set /flir_camera camera.AcquisitionFrameRateEnable true
ros2 param set /flir_camera camera.AcquisitionFrameRate 10.0
ros2 param set /flir_camera camera.ExposureAuto '"Off"'
ros2 param set /flir_camera camera.ExposureTime_FloatVal 8000.0
ros2 param set /flir_camera camera.GainAuto '"Off"'
ros2 param set /flir_camera camera.GainDB_Val 0.0
```

시작할 때 고정값으로 넣고 싶으면 [flir_camera_controls_example.yaml](/home/shchon11/projects/DM/Flir_ROS_Bandwidth_test/src/flir_spinnaker_camera/config/flir_camera_controls_example.yaml)을 복사해서 `params_file`로 넘기면 됩니다.
이 파일에는 FPS, exposure, gain, white balance, gamma, ROI, trigger, stream, tl_device 항목을 주석으로 정리해 뒀습니다.

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  params_file:=/home/shchon11/projects/DM/Flir_ROS_Bandwidth_test/src/flir_spinnaker_camera/config/flir_camera_controls_example.yaml
```

## 참고

- 기본적으로 카메라 한 대를 대상으로 합니다.
- `image_raw`는 카메라에서 받은 원본 포맷을 그대로 퍼블리시합니다.
- `image_rgb`는 같은 프레임을 Spinnaker `ImageProcessor`로 `RGB8` 변환해 퍼블리시합니다.
- `image_rgb/compressed`는 같은 `RGB8` 프레임을 OpenCV로 JPEG/PNG 인코딩한 `sensor_msgs/msg/CompressedImage`입니다.
- 이 카메라에서 확인한 지원 포맷에는 `BayerRG8`, `BayerRG16`, `BayerRG10p`, `BayerRG10Packed`, `BayerRG12p`, `BayerRG12Packed`, `Mono8`, `Mono16`, `Mono10p`, `Mono10Packed`, `Mono12p`, `Mono12Packed`가 포함됩니다.
- ROS `sensor_msgs/Image`는 표준 Bayer 10/12 인코딩 문자열이 없어서, `BayerRG10*`/`BayerRG12*`/`Mono10*`/`Mono12*`로 수신한 raw는 호스트에서 `16-bit`로 풀어 `bayer_*16` 또는 `mono16`으로 퍼블리시합니다.
- GenICam `Command` 타입 노드는 ROS 파라미터가 아니라 실행 명령이어서, 현재 브리지는 `bool/int/float/enum/string` writable 노드만 노출합니다.

## 압축 전후 확인

토픽 타입 확인:

```bash
ros2 topic list -t | grep image_rgb
```

압축 포맷 확인:

```bash
ros2 topic echo /image_rgb/compressed --once --field format
```

대역폭 비교:

```bash
ros2 topic bw /image_rgb
ros2 topic bw /image_rgb/compressed
```

주기 비교:

```bash
ros2 topic hz /image_rgb
ros2 topic hz /image_rgb/compressed
```

실제로 압축 토픽만 쓰고 싶으면 uncompressed RGB를 끄면 됩니다:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py publish_rgb:=false publish_rgb_compressed:=true
```

압축 전후를 동시에 켜두면 비교는 쉽지만, 둘 다 퍼블리시하므로 전체 ROS 트래픽은 오히려 늘어납니다.

## 10/12/16-bit 실행 예시

8-bit raw:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  auto_pixel_format:=false \
  pixel_format:=BayerRG8 \
  publish_rgb:=false \
  publish_rgb_compressed:=false
```

12-bit packed raw:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  auto_pixel_format:=false \
  pixel_format:=BayerRG12p \
  publish_rgb:=false \
  publish_rgb_compressed:=false
```

16-bit raw:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  auto_pixel_format:=false \
  pixel_format:=BayerRG16 \
  publish_rgb:=false \
  publish_rgb_compressed:=false
```

참고:

- `BayerRG12p`로 카메라 링크를 줄여도 ROS의 `/image_raw`는 `bayer_rggb16`으로 퍼블리시되므로 ROS 토픽 크기는 16-bit raw와 같아집니다.
- 즉 10/12 packed의 이득은 주로 `camera -> PC` 링크 대역폭에서 생기고, ROS 내부 토픽까지 줄이려면 packed raw를 별도 타입으로 내보내는 구조가 추가로 필요합니다.
