# AUAV Hybrid Air/Water Firmware

PX4 firmware for a hybrid autonomous vehicle that flies as a quadrotor and
operates on the water surface with a Blue Robotics T500-style horizontal
thruster. Based on PX4 `release/1.17` at commit
`f65ddce53985f13cd4fa24e7c0d78345b467bf1f`.

This repository is for **firmware development** (flight stack, Water mode,
SITL airframe, Gazebo model/world assets). To **build and run** Gazebo SITL,
use the sibling Docker workspace
[`AUAV-sim-ros/sim`](../AUAV-sim-ros/sim).

## Features

- Four aerial motors in a measured 600 mm square layout.
- 15-inch visible propellers and simplified HAUV geometry.
- Two 700 x 100 x 200 mm pontoons with 1.16 m center spacing.
- Sealed 500 x 200 x 140 mm electronics compartment.
- Fifth, bidirectional horizontal T500 actuator.
- Custom PX4 `Water` navigation mode.
- Car-like water steering that reverses while backing up.
- Dry deck and 24 x 16 x 3 m test pool (Gazebo worlds under `Tools/hauv/gz/`).
- Reversible Gazebo ESC output support.

## Repository layout

| Path | Purpose |
|---|---|
| `Tools/hauv/gz/` | Gazebo HAUV model and worlds |
| `src/modules/hauv_water_control/` | Manual Water-mode controller |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4902_gz_hauv_4plus1` | HAUV SITL airframe and actuator allocation |
| `src/modules/simulation/gz_plugins/buoyancy/` | Gazebo buoyancy integration |
| `src/modules/simulation/gz_bridge/` | Gazebo startup and reversible ESC support |

## Run simulation (Docker)

Hướng dẫn đầy đủ (chạy, QGC, git push, publish Docker Hub):
[`AUAV-sim-ros/README.md`](../AUAV-sim-ros/README.md).

```sh
cd ../AUAV-sim-ros/sim
cp .env.example .env
./scripts/test_gz_model.sh
./scripts/build_px4.sh
./scripts/up.sh                 # flight (DART, dry deck)
SIM_MODE=water ./scripts/up.sh  # water (Bullet, pool)
```

In the PX4 shell for water:

```sh
commander mode water
commander arm
```

Center the throttle and right stick before arming. Geometry notes:
[`Tools/hauv/README.md`](Tools/hauv/README.md).

## Water controls

| Input | Function |
|---|---|
| Throttle above 50% | T500 forward thrust |
| Throttle at 50% | T500 stopped |
| Throttle below 50% | T500 reverse thrust |
| Right stick left/right | Water steering |

Steering behaves like a car: forward+left yaws counter-clockwise; reverse+left
yaws clockwise; steering authority fades at neutral thrust. Left-stick yaw is
unused in Water mode.

## Initial Water-mode tuning

```sh
param set HAUV_W_THR_MAX 0.20
param set HAUV_W_YAW_MAX 0.40
param set HAUV_W_Q_IDLE 0.12
param set HAUV_W_THR_SLEW 0.20
param set HAUV_W_YAW_SLEW 0.50
```

## Native host deps (optional)

Only needed if you build outside Docker:

```sh
bash Tools/setup/ubuntu.sh
```

Restart after the setup script if it changes group membership.

## Physics / land-detection notes

- Gazebo 8: use DART for flight and Bullet Featherstone for water; restart when
  switching (`SIM_MODE`).
- Multicopter land detector can report `landed` while floating; auto disarm on
  land is disabled for this development airframe.

## Model status

Provisional: mass/inertia, propeller and T500 curves, buoyancy volume, drag,
air/water transition thresholds. See `Tools/hauv/README.md`.

## Upstream and license

Derived from [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot). BSD 3-Clause
in `LICENSE`.
