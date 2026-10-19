# AUAV Hybrid Air/Water Firmware

PX4 firmware for a hybrid autonomous vehicle that flies as a quadrotor and
operates on the water surface with a Blue Robotics T500-style horizontal
thruster. Based on PX4 `release/1.17`.

This repository is **firmware only** (flight stack, Water mode, SITL airframe,
installed Gazebo asset mirror). To **build and run** Gazebo SITL, use the
sibling Docker workspace
[`AUAV-sim-ros`](https://github.com/thanhlong1003/AUAV-sim-ros).

## Features

- Quadrotor aerial propulsion with CAD-aligned motor poses (airframe `4902`).
- Fifth bidirectional horizontal T500 actuator (center aft bulkhead).
- Custom PX4 **Water** navigation mode (`hauv_water_control`).
- Surface steering via differential quad torque (spot turns; reverses when backing).
- Auto-disarm on land disabled for water splashdown development.
- Gazebo pool / surface worlds (canonical copy in AUAV-sim-ros `gz_standalone`).

## Layout

| Path | Purpose |
|---|---|
| Sibling `AUAV-sim-ros/gz_standalone/` | **Canonical** Gazebo models / worlds |
| `Tools/hauv/gz/` | Installed mirror (`px4_custom_setup.sh` / Docker entrypoint) |
| `src/modules/hauv_water_control/` | Manual Water-mode controller |
| `ROMFS/.../4902_gz_hauv_4plus1` | HAUV SITL airframe / actuator allocation |
| `src/modules/simulation/gz_bridge/` | Prefers `HAUV_GZ_ROOT`, falls back to `Tools/hauv/gz` |

Edit meshes/SDF in **AUAV-sim-ros**, then sync:

```sh
cd ../AUAV-sim-ros/gz_standalone
PX4_DIR=$PWD/../../PX4-Autopilot-VinUni-AUAV ./px4_custom_setup.sh
```

## Run simulation (Docker)

Full guide: [`AUAV-sim-ros/README.md`](../AUAV-sim-ros/README.md).

```sh
cd ../AUAV-sim-ros/sim
cp .env.example .env
./scripts/test_gz_model.sh
./scripts/build_px4.sh
./scripts/up.sh                 # flight (DART, dry deck)
SIM_MODE=water ./scripts/up.sh  # water (Bullet, pool)
```

```sh
commander mode water
# Center throttle (~50%) and sticks, then:
commander arm
```

## Water controls

| Input | Function |
|---|---|
| Throttle > 50% | T500 forward |
| Throttle = 50% | T500 stop |
| Throttle < 50% | T500 reverse |
| Yaw stick **or** right stick L/R | Steering (spot turn OK; reverses when backing) |

## Water-mode tuning

```sh
param set HAUV_W_THR_MAX 0.35
param set HAUV_W_YAW_MAX 1.00
param set HAUV_W_Q_IDLE 0.22
param set HAUV_W_THR_SLEW 0.20
param set HAUV_W_YAW_SLEW 0.50
```

Keep `COM_DISARM_LAND=-1` (baked in airframe `4902`) so splashdown does not
auto-disarm. Surface yaw feel also depends on Gazebo hydro `nR` in
`gz_standalone/.../model.sdf`.

## Physics notes

- Use DART for flight and Bullet Featherstone for water; restart when switching
  `SIM_MODE`.
- Multicopter land detector may report `landed` while floating; auto-disarm is
  disabled for this airframe.
- Prop spin visualization tracks real PX4 motor commands; weak vehicle yaw is a
  **physics** (hydro / authority) issue, not a visual-only one.

## Model status

Provisional: mass/inertia, propeller and T500 curves, buoyancy volume, and
hydro coefficients. Prefer CAD-aligned poses in `frame_align.json` over older
placeholder geometry notes.

## License

Inherit PX4 licenses for upstream code. Project-specific HAUV additions follow
VinUni AUAV team policy.
