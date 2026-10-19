# AUAV Hybrid Air/Water Simulation

PX4 SITL and Gazebo Harmonic simulation for a hybrid autonomous vehicle that
flies as a quadrotor and operates on the water surface with a Blue Robotics
T500-style horizontal thruster.

This project is based on PX4 `release/1.17` at commit
`f65ddce53985f13cd4fa24e7c0d78345b467bf1f`.

## Features

- Four aerial motors in a measured 600 mm square layout.
- 15-inch visible propellers and simplified HAUV geometry.
- Two 700 x 100 x 200 mm pontoons with 1.16 m center spacing.
- Sealed 500 x 200 x 140 mm electronics compartment.
- Fifth, bidirectional horizontal T500 actuator.
- Custom PX4 `Water` navigation mode.
- Car-like water steering that reverses while backing up.
- Dry deck and 24 x 16 x 3 m test pool.
- Separate stable Gazebo profiles for flight and water testing.
- Reversible Gazebo ESC output support.

## Repository layout

| Path | Purpose |
|---|---|
| `Tools/hauv/` | Launcher, tests, model, worlds and detailed notes |
| `src/modules/hauv_water_control/` | Manual Water-mode controller |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4902_gz_hauv_4plus1` | HAUV airframe and actuator allocation |
| `src/modules/simulation/gz_plugins/buoyancy/` | Gazebo buoyancy integration |
| `src/modules/simulation/gz_bridge/` | Gazebo startup and reversible ESC support |

## Requirements

- Ubuntu Linux supported by PX4 `release/1.17`.
- PX4 development dependencies.
- Gazebo Harmonic (`gz-sim8`); the tested version is 8.11.0.
- QGroundControl for telemetry and optional USB joystick input.

**Recommended:** run Gazebo SITL via the sibling Docker workspace
[`AUAV-sim-ros/sim`](../AUAV-sim-ros/sim) so the host only edits this firmware
tree. Build and spawn happen inside the container (bind-mount).

Native host install (optional):

```sh
bash Tools/setup/ubuntu.sh
```

Restart the computer after the setup script if it changes group membership or
installs system components.

## Clone

```sh
git clone --recursive https://github.com/thanhlong1003/AUAV-sim-ros.git
cd AUAV-sim-ros
```

If the repository was cloned without submodules:

```sh
git submodule update --init --recursive
```

## Build and validate

```sh
make -j2 px4_sitl_default
bash Tools/hauv/test_gz_model.sh
```

The validation script checks the SDF files, actuator geometry, Water-mode
registration, propulsion constants, buoyancy setup and airframe mapping.

## Run

### Flight profile

```sh
bash Tools/hauv/run_gz_hauv.sh
```

This is also available explicitly as:

```sh
bash Tools/hauv/run_gz_hauv.sh flight
```

The flight profile uses DART physics and starts on the south dry deck at
`0,-12,0.6`. Use normal PX4 flight modes such as Position mode.

### Water profile

```sh
bash Tools/hauv/run_gz_hauv.sh water
```

The water profile uses Bullet Featherstone physics and starts at the pool's
calm-water equilibrium. In the PX4 shell:

```sh
commander mode water
commander arm
```

Center the throttle and right stick before arming.

## Water controls

| Input | Function |
|---|---|
| Throttle above 50% | T500 forward thrust |
| Throttle at 50% | T500 stopped |
| Throttle below 50% | T500 reverse thrust |
| Right stick left/right | Water steering |

Steering behaves like a car:

- forward plus left produces counter-clockwise yaw;
- reverse plus left produces clockwise yaw;
- steering authority fades to zero at neutral thrust.

The normal left-stick yaw input is unused in Water mode. Roll and pitch water
stabilization are future work.

## Initial Water-mode tuning

Parameters can be changed live from the PX4 shell:

```sh
param set HAUV_W_THR_MAX 0.20
param set HAUV_W_YAW_MAX 0.40
param set HAUV_W_Q_IDLE 0.12
param set HAUV_W_THR_SLEW 0.20
param set HAUV_W_YAW_SLEW 0.50
```

`HAUV_W_Q_IDLE` provides differential-yaw headroom. Increase it gradually and
stop if the vehicle starts lifting or rocking on the water.

## USB radio and QGroundControl

The transmitter must expose a USB HID joystick. In QGroundControl:

1. Connect PX4 SITL.
2. Open **Vehicle Setup > Joystick**.
3. Select and calibrate the controller.
4. Keep joystick mode set to Normal.

PX4 uses `COM_RC_IN_MODE=1` for joystick input on this airframe. Confirm the
normalized controls with:

```sh
listener manual_control_setpoint -n 10
```

For bidirectional thrust, the expected throttle values are approximately `-1`
at the bottom, `0` at the center and `1` at the top.

## Physics limitation

Gazebo 8 does not currently provide both stable DART buoyancy and stable Bullet
rotor-joint flight for this hybrid model in one session. The launcher therefore
uses:

- DART for multicopter flight and Position mode;
- Bullet Featherstone for buoyancy and Water mode.

Restart the simulator when switching profiles.

## Land detection limitation

PX4's multicopter land detector does not know about water. A stationary floating
vehicle can satisfy its low-velocity, low-acceleration and low-thrust tests and
be reported as landed. Automatic landed disarming is disabled for this
development airframe, but `vehicle_land_detected` can still report `landed` on
water. A dedicated water-contact sensor and transition state machine are future
work.

## Model status

The model is intended for software integration and gross behavior testing. The
following values remain provisional:

- all-up mass and inertia tensor;
- 15-inch propeller thrust and reaction-torque curves;
- T500 voltage, thrust curve and installation losses;
- displaced volume and center of buoyancy;
- hydrodynamic drag coefficients;
- automatic air/water transition thresholds.

See `Tools/hauv/README.md` for detailed geometry, actuator mapping and test
notes.

## Upstream and license

This repository is derived from
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot). PX4 and these changes are
distributed under the BSD 3-Clause license in `LICENSE`.
