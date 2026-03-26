# flir_spinnaker_camera

ROS 2 Humble package for FLIR/Spinnaker cameras.

Published topics:

- `image_raw`
- `camera_info`
- `image_raw/metadata`
- `image_rgb/compressed`

The node publishes raw images, default `camera_info`, compressed RGB images, and a separate
metadata topic for per-frame capture information such as pixel format, frame
ID, timestamps, exposure, gain, gamma, black level, and frame-rate related
settings.

`camera_info` can be populated from YAML through `camera_info.distortion_model`,
`camera_info.d`, `camera_info.k`, `camera_info.r`, `camera_info.p`,
`camera_info.binning_*`, and `camera_info.roi.*`.

For `camera_info.d`, keep explicit numeric values in YAML. ROS 2 parameter files
do not infer a typed `double[]` from an empty list like `[]`.
