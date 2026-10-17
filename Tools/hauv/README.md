# HAUV Gazebo SITL

The HAUV model supports normal PX4 multicopter flight modes and a dedicated
manual `Water` mode for surface operation.

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

## Run

```sh
bash Tools/hauv/test_gz_model.sh
bash Tools/hauv/run_gz_hauv.sh
```

The launcher uses the `hauv_pool` world by default. It contains a 24 x 16 x 3 m
pool surrounded by a dry 50 x 36 m deck. The vehicle starts on the south deck
at `0,-12,0.6`. To use the original open-water world:

```sh
PX4_GZ_WORLD=hauv_surface bash Tools/hauv/run_gz_hauv.sh water
```

To start near the calm-water equilibrium instead of dropping the vehicle from
above the surface:

```sh
bash Tools/hauv/run_gz_hauv.sh water
```

The simplified model uses a lowered center of mass and provisional
hydrodynamic drag for passive surface stability.

## Water mode

Start at the calm-water equilibrium:

```sh
bash Tools/hauv/run_gz_hauv.sh water
```

The launcher uses Gazebo's DART backend by default because it provides stable
multicopter flight and Position mode. The `water` argument selects Bullet and
the pool spawn automatically. Running without an argument selects DART and the
dry flight spawn.

Gazebo 8 currently cannot provide both stable DART buoyancy and stable Bullet
rotor-joint flight for this hybrid model in one simulator session.

Then use the PX4 shell:

```sh
commander mode water
commander arm
```

Put the throttle stick at its 50% center detent before arming. Water mode waits
for normalized throttle and right-stick horizontal inputs near `0.0` before it
accepts commands.

The upper half of the throttle stick controls forward T500 thrust and the lower
half controls reverse thrust. Right-stick horizontal controls differential
aerial-motor yaw with car-like steering: a left command yaws counter-clockwise
while moving forward and clockwise while reversing. Steering authority fades
to zero as T500 thrust approaches neutral. The normal left-stick yaw axis is
unused in Water mode. Pitch stabilization is not implemented yet. The four
aerial motors remain at the `HAUV_W_Q_IDLE` collective while armed, which defaults to `0.12`
for stronger differential-yaw headroom; automatic
preflight and landed disarming are disabled for this development airframe.

After arming, Water mode waits for both the throttle and right-stick horizontal
axis to be centered before accepting commands. T500 thrust and yaw torque are
then slew-limited to avoid destabilizing the Gazebo physics engine.

Useful initial tuning parameters are:

```sh
param set HAUV_W_THR_MAX 0.20
param set HAUV_W_YAW_MAX 0.40
param set HAUV_W_Q_IDLE 0.12
param set HAUV_W_THR_SLEW 0.20
param set HAUV_W_YAW_SLEW 0.50
```

Switch back to the normal flight controller with `commander mode posctl`.
Disarm before moving the vehicle by hand.

To test only the T500 output mapping while disarmed:

```sh
actuator_test set -f 105 -v 0.25 -t 2
```

Then verify the four aerial outputs one at a time with functions `101` through
`104`. Keep values low and time-limited.

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
