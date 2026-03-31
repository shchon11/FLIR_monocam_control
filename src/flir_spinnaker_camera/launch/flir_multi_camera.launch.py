from copy import deepcopy
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_bool(value):
    if isinstance(value, bool):
        return value

    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Expected a boolean value, got: {value}")


def _load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _resolve_path(config_path: Path, value: str) -> str:
    if value == "":
        return ""

    candidate = Path(value)
    if candidate.is_absolute():
        return str(candidate)

    config_relative = (config_path.parent / candidate).resolve()
    if config_relative.exists():
        return str(config_relative)

    return str((Path.cwd() / candidate).resolve())


def _load_shared_camera_parameters(params_path: Path):
    loaded = _load_yaml(params_path)

    for key in ("/**", "flir_camera"):
        section = loaded.get(key)
        if isinstance(section, dict) and isinstance(section.get("ros__parameters"), dict):
            return deepcopy(section["ros__parameters"])

    raise RuntimeError(
        f"Could not find a ros__parameters block for '/**' or 'flir_camera' in {params_path}"
    )


def _build_camera_node(camera_name: str, camera_config: dict, shared_parameters: dict, config_path: Path):
    parameters = deepcopy(shared_parameters)

    serial = str(camera_config.get("serial", "")).strip()
    if serial != "":
        parameters["camera_serial"] = serial
    elif "camera_index" in camera_config:
        parameters["camera_index"] = int(camera_config["camera_index"])

    if "camera_index_ordering" in camera_config:
        parameters["camera_index_ordering"] = str(camera_config["camera_index_ordering"])

    parameters["frame_id"] = str(
        camera_config.get("frame_id", f"{camera_name}_optical_frame")
    )

    intrinsic_file = str(camera_config.get("intrinsic_file", "")).strip()
    if intrinsic_file != "":
        parameters["camera_info.yaml_path"] = _resolve_path(config_path, intrinsic_file)
    else:
        parameters["camera_info.yaml_path"] = ""
        if serial != "":
            inferred_intrinsic = Path(
                _resolve_path(config_path, f"calibration/intrinsics/{serial}.yaml")
            )
            if inferred_intrinsic.exists():
                parameters["camera_info.yaml_path"] = str(inferred_intrinsic)

    parameter_overrides = camera_config.get("parameter_overrides", {})
    if parameter_overrides:
        if not isinstance(parameter_overrides, dict):
            raise RuntimeError(
                f"Expected flir_system.cameras.{camera_name}.parameter_overrides to be a mapping"
            )
        parameters.update(deepcopy(parameter_overrides))

    namespace = str(camera_config.get("namespace", camera_name))

    return Node(
        package="flir_spinnaker_camera",
        executable="flir_spinnaker_camera_node",
        name="flir_camera",
        namespace=namespace,
        output="screen",
        parameters=[parameters],
    )


def _build_extrinsics_nodes(system_config: dict, config_path: Path):
    if not _parse_bool(system_config.get("publish_extrinsics", True)):
        return []

    extrinsics_file = str(system_config.get("extrinsics_file", "")).strip()
    if extrinsics_file == "":
        return []

    extrinsics_path = Path(_resolve_path(config_path, extrinsics_file))
    if not extrinsics_path.exists():
        raise RuntimeError(f"Extrinsics file not found: {extrinsics_path}")

    loaded = _load_yaml(extrinsics_path)
    extrinsics = loaded.get("extrinsics", {})
    if not isinstance(extrinsics, dict):
        raise RuntimeError(f"Expected 'extrinsics' to be a mapping in {extrinsics_path}")

    cameras = extrinsics.get("cameras", {})
    if not isinstance(cameras, dict):
        raise RuntimeError(f"Expected 'extrinsics.cameras' to be a mapping in {extrinsics_path}")

    nodes = []
    for camera_name, transform in cameras.items():
        if not isinstance(transform, dict):
            raise RuntimeError(
                f"Expected extrinsics.cameras.{camera_name} to be a mapping in {extrinsics_path}"
            )

        parent = str(transform.get("parent", "")).strip()
        child = str(transform.get("child", "")).strip()
        translation = transform.get("translation_xyz", [0.0, 0.0, 0.0])
        rotation = transform.get("rotation_rpy", [0.0, 0.0, 0.0])

        if parent == "" or child == "":
            raise RuntimeError(
                f"Extrinsics for {camera_name} must define both parent and child in {extrinsics_path}"
            )

        if len(translation) != 3 or len(rotation) != 3:
            raise RuntimeError(
                f"Extrinsics for {camera_name} must define 3-element translation_xyz and rotation_rpy"
            )

        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{camera_name}_extrinsics",
                output="screen",
                arguments=[
                    str(float(translation[0])),
                    str(float(translation[1])),
                    str(float(translation[2])),
                    str(float(rotation[0])),
                    str(float(rotation[1])),
                    str(float(rotation[2])),
                    parent,
                    child,
                ],
            )
        )

    return nodes


def _build_nodes(context):
    config_path = Path(LaunchConfiguration("config_file").perform(context)).resolve()
    loaded = _load_yaml(config_path)

    system_config = loaded.get("flir_system", {})
    if not isinstance(system_config, dict):
        raise RuntimeError(f"Expected 'flir_system' to be a mapping in {config_path}")

    shared_params_file = str(system_config.get("shared_camera_params_file", "flir_camera.yaml"))
    shared_params_path = Path(_resolve_path(config_path, shared_params_file))
    shared_parameters = _load_shared_camera_parameters(shared_params_path)

    shared_parameter_overrides = system_config.get("shared_parameter_overrides", {})
    if shared_parameter_overrides:
        if not isinstance(shared_parameter_overrides, dict):
            raise RuntimeError(
                f"Expected 'flir_system.shared_parameter_overrides' to be a mapping in {config_path}"
            )
        shared_parameters.update(deepcopy(shared_parameter_overrides))

    cameras = system_config.get("cameras", {})
    if not isinstance(cameras, dict):
        raise RuntimeError(f"Expected 'flir_system.cameras' to be a mapping in {config_path}")

    nodes = []
    for camera_name, camera_config in cameras.items():
        if not isinstance(camera_config, dict):
            raise RuntimeError(f"Expected flir_system.cameras.{camera_name} to be a mapping")

        if not _parse_bool(camera_config.get("enabled", True)):
            continue

        nodes.append(_build_camera_node(camera_name, camera_config, shared_parameters, config_path))

    nodes.extend(_build_extrinsics_nodes(system_config, config_path))

    if not nodes:
        raise RuntimeError(f"No enabled cameras were configured in {config_path}")

    return nodes


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("flir_spinnaker_camera"), "config", "flir_cameras.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="Inventory YAML for the multi-camera FLIR system.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
