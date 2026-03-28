# flir_camera_calibration

ROS 2 Humble calibration helper for the FLIR workspace.

입력:

- `/image_rgb/compressed` 구독

출력:

- 체스보드가 검출되면 코너를 그려 넣은 `/calibration/image_annotated/compressed`

기능:

- OpenCV로 compressed JPEG/PNG 디코드
- 구독된 프레임마다 바로 저해상도 preview에서 체스보드 검출 및 코너 오버레이
- OpenCV 창에서 `space`를 눌러 샘플 캡처
- `c`를 눌러 `cv::calibrateCamera()` 실행 후 YAML 저장
- `r`로 캡처 리셋
- `q` 또는 `Esc`로 종료

중요:

- `board_cols`, `board_rows`는 체스보드의 `inner corners` 기준
- live preview는 성능을 위해 downscale + FAST_CHECK를 쓰고, `space` 캡처 시에는 원본 full-resolution에서 다시 정밀 검출
- preview는 별도 FPS 제한 없이 입력 compressed 이미지가 들어오는대로 바로 갱신됨
- 기본 live preview 검출 폭은 `640px`이고, 보드가 너무 작게 보이면 `preview_max_width`를 올리면 됨
- 캘리브레이션 저장 파일은 FLIR 카메라 노드의 `camera_info.*` 파라미터 형태로 생성됨
- FLIR 카메라 노드에서 `camera_info.yaml_path:=<saved_yaml>` 로 주면 저장된 결과가 `/camera_info`에 그대로 반영됨
- 원본 캡처 이미지는 `sample_image_dir`가 비어 있지 않으면 같이 저장되고, 기본 `calibration/captures/` 이미지는 gitignore 처리됨

## 빌드

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_camera_calibration
```

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

## 기본 저장 위치

- calibration YAML: `calibration/flir_camera_info.yaml`
- 캡처 이미지: `calibration/captures/`
