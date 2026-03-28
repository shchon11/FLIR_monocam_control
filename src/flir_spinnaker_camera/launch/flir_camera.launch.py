from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Expected a boolean launch value, got: {value}")


def _optional_override(context, name, parser=None):
    value = LaunchConfiguration(name).perform(context)
    if value == "":
        return None
    return parser(value) if parser is not None else value


def _build_camera_node(context):
    parameter_overrides = {}

    for name in (
        "camera_serial",
        "frame_id",
        "pixel_format",
        "rgb_compression_format",
        "publisher_qos_reliability",
        "camera_info_yaml_path",
    ):
        value = _optional_override(context, name)
        if value is not None:
            if name == "camera_info_yaml_path":
                parameter_overrides["camera_info.yaml_path"] = value
            else:
                parameter_overrides[name] = value

    for name in (
        "auto_pixel_format",
        "publish_camera_info",
        "publish_metadata",
        "publish_rgb_compressed",
        "use_camera_timestamp_in_header",
    ):
        value = _optional_override(context, name, _parse_bool)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("rgb_jpeg_quality", "rgb_png_compression_level", "publisher_qos_depth"):
        value = _optional_override(context, name, int)
        if value is not None:
            parameter_overrides[name] = value

    return [
        Node(
            package="flir_spinnaker_camera",
            executable="flir_spinnaker_camera_node",
            name="flir_camera",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                parameter_overrides,
            ],
        )
    ]


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("flir_spinnaker_camera"), "config", "flir_camera.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the FLIR camera node.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Path to the ROS 2 parameter file.",
            ),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="",
                description="Optional FLIR camera serial number.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Optional frame_id override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "camera_info_yaml_path",
                default_value="",
                description="Optional path to a calibration YAML whose camera_info.* values override the defaults.",
            ),
            DeclareLaunchArgument(
                "publisher_qos_reliability",
                default_value="",
                description="Optional publisher QoS reliability override: reliable or best_effort.",
            ),
            DeclareLaunchArgument(
                "auto_pixel_format",
                default_value="",
                description="Optional auto_pixel_format override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "use_camera_timestamp_in_header",
                default_value="",
                description="Optional header timestamp source override. True uses camera timestamps when available.",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="",
                description="Optional camera PixelFormat override such as BayerRG8, BayerRG12p, or BayerRG16.",
            ),
            DeclareLaunchArgument(
                "publish_camera_info",
                default_value="",
                description="Optional publish_camera_info override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "publish_metadata",
                default_value="",
                description="Optional publish_metadata override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "publish_rgb_compressed",
                default_value="",
                description="Optional publish_rgb_compressed override. Empty keeps the YAML value.",
            ),
            DeclareLaunchArgument(
                "rgb_compression_format",
                default_value="",
                description="Optional compressed RGB format override: jpeg or png.",
            ),
            DeclareLaunchArgument(
                "rgb_jpeg_quality",
                default_value="",
                description="Optional JPEG quality override for /image_rgb/compressed.",
            ),
            DeclareLaunchArgument(
                "rgb_png_compression_level",
                default_value="",
                description="Optional PNG compression level override for /image_rgb/compressed.",
            ),
            DeclareLaunchArgument(
                "publisher_qos_depth",
                default_value="",
                description="Optional publisher QoS queue depth override.",
            ),
            OpaqueFunction(function=_build_camera_node),
        ]
    )
