# flir_camera_calibration

체스보드 기반 캘리브레이션을 위한 보조 패키지다.

## 역할

- `/image_rgb/compressed`를 받아 체스보드 검출
- 검출된 보드를 preview로 보여주고 annotated compressed 토픽 퍼블리시
- 샘플을 모아 calibration YAML 저장

## 입력과 출력

입력:

- `/image_rgb/compressed`

출력:

- `/calibration/image_annotated/compressed`
- `calibration/flir_camera_info.yaml`
- `calibration/captures/` 내부 캡처 이미지

## 조작 키

- `space`: 현재 보드 샘플 캡처
- `c`: `cv::calibrateCamera()` 실행 후 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

## 동작 방식

- preview는 입력 프레임이 들어오는대로 바로 갱신된다.
- live preview 검출은 성능 때문에 downscale + FAST_CHECK를 쓴다.
- 실제 캡처 시에는 원본 full-resolution에서 다시 정밀 검출한다.
- 기본 preview 검출 폭은 `640px`이다.

## 실행

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_camera_calibration calibration.launch.py
```

체스보드 크기 override 예:

```bash
ros2 launch flir_camera_calibration calibration.launch.py \
  board_cols:=9 \
  board_rows:=6 \
  square_size_m:=0.024
```

## 결과 활용

저장된 `calibration/flir_camera_info.yaml`은 `flir_spinnaker_camera`의 `camera_info.yaml_path`에 바로 연결할 수 있다.

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

## 메모

- `board_cols`, `board_rows`는 square 개수가 아니라 `inner corners` 기준이다.
- `sample_image_dir`가 비어 있지 않으면 캡처 원본도 저장된다.
- 기본 `calibration/captures/` 이미지는 gitignore 처리되어 있다.
