# flir_camera_calibration

체스보드 기반 intrinsic calibration helper다.

## 역할

- `/image_rgb/compressed` 구독
- 보드가 보이면 annotated preview 퍼블리시
- 샘플 캡처 후 intrinsic YAML 저장

## 입력과 출력

입력:

- `image_rgb/compressed`

출력:

- `calibration/image_annotated/compressed`
- `calibration/intrinsics/<serial>.yaml`
- `calibration/captures/`

namespace를 주면 입력/출력도 같이 namespace를 탄다.

예:

- `/camera0/image_rgb/compressed`
- `/camera0/calibration/image_annotated/compressed`

## 실행

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_camera_calibration calibration.launch.py namespace:=camera0
```

## 조작 키

- `space`: 현재 보드 샘플 캡처
- `c`: `cv::calibrateCamera()` 실행 후 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

## 저장 규칙

- 가능하면 sibling `flir_camera` 노드에서 실제 선택된 serial을 읽음
- 저장 파일은 `calibration/intrinsics/<serial>.yaml`
- serial을 못 읽는 경우에만 `output_yaml_path` fallback 사용
- saved intrinsic YAML은 로컬 calibration artifact로 보고 gitignore 처리한다

## 메모

- `board_cols`, `board_rows`는 square 개수가 아니라 `inner corners` 기준이다.
- live preview는 downscale + FAST_CHECK를 사용한다.
- 실제 캡처 시에는 full-resolution에서 다시 정밀 검출한다.
- `calibration/captures/`는 gitignore 처리되어 있다.
