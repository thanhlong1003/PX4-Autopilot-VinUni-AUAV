#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
px4_root=$(cd -- "${script_dir}/../.." && pwd)
model="${px4_root}/Tools/hauv/gz/models/hauv_4plus1/model.sdf"
surface_world="${px4_root}/Tools/hauv/gz/worlds/hauv_surface.sdf"
pool_world="${px4_root}/Tools/hauv/gz/worlds/hauv_pool.sdf"
airframe="${px4_root}/ROMFS/px4fmu_common/init.d-posix/airframes/4902_gz_hauv_4plus1"

python3 - "${model}" "${surface_world}" "${pool_world}" <<'PY'
import sys
import xml.etree.ElementTree as ET
for path in sys.argv[1:]:
    ET.parse(path)
    print(f"XML OK: {path}")
PY

for expected in \
    "CA_ROTOR_COUNT 5" \
    "CA_ROTOR0_PX  0.30" \
    "CA_ROTOR0_PY  0.30" \
    "CA_ROTOR0_KM  0.08" \
    "CA_ROTOR1_PX -0.30" \
    "CA_ROTOR1_PY -0.30" \
    "CA_ROTOR1_KM  0.08" \
    "CA_ROTOR2_PX  0.30" \
    "CA_ROTOR2_PY -0.30" \
    "CA_ROTOR2_KM -0.08" \
    "CA_ROTOR3_PX -0.30" \
    "CA_ROTOR3_PY  0.30" \
    "CA_ROTOR3_KM -0.08" \
    "CA_ROTOR4_AX  1.00" \
    "CA_ROTOR4_AZ  0.00" \
    "CA_R_REV 16" \
    "SIM_GZ_EC_FUNC1 101" \
    "SIM_GZ_EC_FUNC2 102" \
    "SIM_GZ_EC_FUNC3 103" \
    "SIM_GZ_EC_FUNC4 104" \
    "SIM_GZ_EC_FUNC5 105"
do
    grep -q "${expected}" "${airframe}"
done

grep -q 'hauv_water_control start' "${airframe}"
grep -q 'COM_RC_IN_MODE 1' "${airframe}"
grep -q 'COM_DISARM_LAND -1' "${airframe}"
grep -q 'COM_DISARM_PRFLT -1' "${airframe}"
grep -q 'NAVIGATION_STATE_WATER = 16' \
    "${px4_root}/msg/versioned/VehicleStatus.msg"
grep -q 'CONFIG_MODULES_HAUV_WATER_CONTROL=y' \
    "${px4_root}/boards/px4/sitl/default.px4board"
grep -q 'PARAM_DEFINE_FLOAT(HAUV_W_Q_IDLE, 0.12f)' \
    "${px4_root}/src/modules/hauv_water_control/hauv_water_control_params.c"
grep -q 'PARAM_DEFINE_FLOAT(HAUV_W_THR_SLEW, 0.20f)' \
    "${px4_root}/src/modules/hauv_water_control/hauv_water_control_params.c"
grep -q 'PARAM_DEFINE_FLOAT(HAUV_W_YAW_MAX, 0.40f)' \
	"${px4_root}/src/modules/hauv_water_control/hauv_water_control_params.c"
grep -q 'PARAM_DEFINE_FLOAT(HAUV_W_YAW_SLEW, 0.50f)' \
	"${px4_root}/src/modules/hauv_water_control/hauv_water_control_params.c"

grep -q '<motorNumber>4</motorNumber>' "${model}"
grep -q '<jointName>t500_joint</jointName>' "${model}"
grep -q '<pose>0 0.58 0 0 0 0</pose>' "${model}"
grep -q '<pose>0 -0.58 0 0 0 0</pose>' "${model}"
grep -q '<box><size>0.70 0.10 0.20</size></box>' "${model}"
grep -q '<box><size>0.50 0.20 0.14</size></box>' "${model}"
grep -q '<box><size>0.381 0.028 0.004</size></box>' "${model}"
grep -q '<cylinder><radius>0.012</radius><length>1.16</length></cylinder>' "${model}"
grep -q '<linear>0.80</linear>' "${model}"
grep -q '<angular>2.00</angular>' "${model}"
test "$(grep -c '<rotorVelocitySlowdownSim>100</rotorVelocitySlowdownSim>' "${model}")" -eq 5
test "$(grep -c '<motorType>velocity</motorType>' "${model}")" -eq 5
test "$(grep -c '<momentConstant>0.08</momentConstant>' "${model}")" -eq 4
! grep -q 'name="rotor_[0-3]_collision"' "${model}"
! grep -q 'name="t500_collision"' "${model}"
grep -q 'filename="gz-sim-hydrodynamics-system"' "${model}"
grep -q '<zW>-40.0</zW>' "${model}"
grep -q '<above_depth>0</above_depth>' "${surface_world}"
grep -q '<above_depth>0</above_depth>' "${pool_world}"
grep -q 'name="pool_floor"' "${pool_world}"
grep -q 'name="pool_wall_west"' "${pool_world}"
grep -q '<size>24 16</size>' "${pool_world}"
! grep -q 'pontoon_left_joint' "${model}"
! grep -q 'pontoon_right_joint' "${model}"
! grep -q '<enable>hauv_4plus1_0::pontoon_' "${pool_world}"
grep -q '4902_gz_hauv_4plus1' \
    "${px4_root}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"

echo "PASS: HAUV model, worlds, Water mode and actuator mapping are internally consistent."
