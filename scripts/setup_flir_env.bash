#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "source this file instead of executing it: source scripts/setup_flir_env.bash" >&2
  exit 1
fi

_flir_env_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FLIR_ROS_WS="$(cd "${_flir_env_script_dir}/.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    echo "[flir_env] ROS 2 Humble setup script not found at /opt/ros/humble/setup.bash" >&2
    return 1
  fi
fi

if [[ -z "${SPINNAKER_ROOT:-}" && -d /opt/spinnaker ]]; then
  export SPINNAKER_ROOT=/opt/spinnaker
fi

if [[ -f "${FLIR_ROS_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${FLIR_ROS_WS}/install/setup.bash"
else
  echo "[flir_env] ${FLIR_ROS_WS}/install/setup.bash not found yet. Build the workspace first." >&2
fi

if [[ -z "${SPINNAKER_ROOT:-}" ]]; then
  echo "[flir_env] SPINNAKER_ROOT is not set. Export it if your Spinnaker SDK is installed outside the default search paths." >&2
fi

unset _flir_env_script_dir
