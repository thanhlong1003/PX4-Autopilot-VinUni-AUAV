#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
px4_root=$(cd -- "${script_dir}/../.." && pwd)
simulation_mode=${1:-flight}

case "${simulation_mode}" in
    flight)
        default_physics=gz-physics-dartsim-plugin
        default_pose=0,-12,0.6,0,0,0
        ;;
    water)
        default_physics=gz-physics-bullet-featherstone-plugin
        default_pose=0,0,0.06,0,0,0
        ;;
    *)
        echo "Usage: $0 [flight|water]" >&2
        exit 2
        ;;
esac

if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: 'gz' was not found. Install Gazebo Harmonic (gz-sim8)." >&2
    exit 1
fi

gz_version=$(gz sim --versions 2>/dev/null | head -n 1 | tr -d ' ')
world=${PX4_GZ_WORLD:-hauv_pool}
world_file="${px4_root}/Tools/hauv/gz/worlds/${world}.sdf"

if [ ! -f "${world_file}" ]; then
    echo "ERROR: HAUV world not found: ${world_file}" >&2
    exit 1
fi

echo "Gazebo version: ${gz_version:-unknown}"
echo "World: ${world}"
echo "Mode: ${simulation_mode}"
echo "Physics: ${PX4_GZ_SIM_PHYSICS_ENGINE:-${default_physics}}"
echo "Starting HAUV 4+1 at ${PX4_GZ_MODEL_POSE:-${default_pose}}"

cd "${px4_root}"
PX4_GZ_SIM_PHYSICS_ENGINE=${PX4_GZ_SIM_PHYSICS_ENGINE:-${default_physics}} \
PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:-${default_pose}} \
    make px4_sitl "gz_hauv_4plus1_${world}"
