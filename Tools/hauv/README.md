# HAUV Gazebo SITL assets

Gazebo models and worlds for the hybrid air/water vehicle. Firmware modules and
the SITL airframe live in this PX4 tree; **launchers and validation scripts**
live in the sibling [`AUAV-sim-ros/sim`](../../../AUAV-sim-ros/sim) workspace.

## Current mapping

| Gazebo ESC channel | PX4 output function | Simulated actuator |
|---:|---:|---|
| 1 | Motor 1 (`101`) | aerial rotor 0 |
| 2 | Motor 2 (`102`) | aerial rotor 1 |
| 3 | Motor 3 (`103`) | aerial rotor 2 |
| 4 | Motor 4 (`104`) | aerial rotor 3 |
| 5 | Motor 5 (`105`) | horizontal T500 |

`CA_ROTOR_COUNT` is 5. The fifth rotor has a body-X thrust axis, so normal
Position mode leaves it stopped while Water mode commands it as forward
propulsion. Water yaw is allocated as differential torque across aerial motors
1-4.

## Simplified geometry

- motor centers form a 0.60 x 0.60 m square;
- aerial propeller diameter is 0.381 m (15 inches);
- pontoon centers are 1.16 m apart after widening the crossbars;
- each pontoon is 0.70 x 0.10 x 0.20 m;
- the sealed electronics compartment is 0.50 x 0.20 x 0.14 m;
- both pontoons and the electronics compartment contribute buoyancy.

The widened layout leaves 39.5 mm between each propeller disc and its nearest
pontoon, and 219 mm between adjacent propeller discs. Motor thrust and torque
constants remain provisional until measured 15-inch propeller data is supplied.

## Run and validate

Use the Docker workflow in `AUAV-sim-ros/sim`:

```sh
cd ../AUAV-sim-ros/sim
./scripts/test_gz_model.sh
./scripts/build_px4.sh
./scripts/up.sh                 # flight
SIM_MODE=water ./scripts/up.sh  # water
```

Default world is `hauv_pool` (24 x 16 x 3 m pool, dry deck). Open-water:

```sh
PX4_GZ_WORLD=hauv_surface SIM_MODE=water ./scripts/up.sh
```

## Water mode (firmware)

In the PX4 shell after spawning with `SIM_MODE=water`:

```sh
commander mode water
commander arm
```

Center throttle and right stick before arming. Throttle above/below 50% drives
the T500 forward/reverse; right-stick horizontal is car-like steering. Tuning
params: `HAUV_W_THR_MAX`, `HAUV_W_YAW_MAX`, `HAUV_W_Q_IDLE`, `HAUV_W_THR_SLEW`,
`HAUV_W_YAW_SLEW`.

Disarmed actuator check:

```sh
actuator_test set -f 105 -v 0.25 -t 2
```

## Provisional parameters (TBD)

- all-up mass and inertia tensor, including CPS05;
- measured mass distribution and inertia tensor;
- propeller size and X4120 thrust/torque curves;
- T500 voltage, thrust curve, reverse behavior and installation losses;
- hull displaced volume, center of buoyancy and hydrodynamic drag;
- water contact sensing and the Air/Water transition thresholds.

The current model uses an approximately 8 kg mass and the measured dimensions
listed above. Mass distribution, fluid drag and propulsion constants remain
provisional and are only suitable for checking wiring and gross behavior.
