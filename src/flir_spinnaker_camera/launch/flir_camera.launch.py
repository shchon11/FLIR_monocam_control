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

    for name in ("camera_serial", "frame_id", "pixel_format", "rgb_compression_format"):
        value = _optional_override(context, name)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("auto_pixel_format", "publish_camera_info", "publish_metadata", "publish_rgb_compressed"):
        value = _optional_override(context, name, _parse_bool)
        if value is not None:
            parameter_overrides[name] = value

    for name in ("rgb_jpeg_quality", "rgb_png_compression_level"):
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
                "auto_pixel_format",
                default_value="",
                description="Optional auto_pixel_format override. Empty keeps the YAML value.",
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
            OpaqueFunction(function=_build_camera_node),
        ]
    )
