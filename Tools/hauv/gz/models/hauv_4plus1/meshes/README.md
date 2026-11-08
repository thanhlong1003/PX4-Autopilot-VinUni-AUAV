# HAUV CAD meshes (V2)

## Airframe

From `HAUV-v2-design.step` product **Big** → assembly export.
SunnySky motor assemblies are excluded; rotor poses in `frame_align.json`.

| File | Role |
|---|---|
| `hauv_big.stl` | Visual mesh (meters, yaw +90°) |
| `hauv_big_collision.stl` | Convex-hull collision mesh |
| `frame_align.json` | Rotor + T200 poses (FLU, meters) |

```sh
cd AUAV-sim-ros/sim/cad && STEP_PATH=../../../HAUV-v2-design.step ./export_big.sh
```

## Propulsion (22×6 + T200)

From `HAUV-v2-design.step` (see `sim/cad/export_big_from_step.py`):

| File | Role |
|---|---|
| `prop_22x6_ccw.stl` / `prop_22x6_cw.stl` | Aerial props (spin about +Z) |
| `motor_sunnysky.stl` | SunnySky X6212S body (physics = X6215S) |
| `t200_thruster.stl` | T200 body (aft bulkhead, fixed on `base_link`) |
| `propulsion_x6215.json` | X6215S LUT / motor constants |
| `propulsion_meshes.json` | Extents / regenerate metadata |
| `propulsion_physics.json` | Datasheet → Gazebo `motorConstant` / `maxRotVelocity` |
| `hydro_physics.json` | Mass / buoyancy / Fossen drag (geometry + wave Cd) |
| `battery_physics.json` | 12S6P Samsung 50S 30 Ah pack + SITL notes |

Physics (Gazebo `thrust = motorConstant · ω²`):

| Actuator | Source | ω_max | T_max | motorConstant |
|---|---|---|---|---|
| Aerial ×4 | X6215S + 22×6.6 @ 50 V / 60 A | 815 rad/s | ~114 N | `1.72e-4` |
| Horizontal | T200 @ 20 V | 426 rad/s | ~66 N | `3.62e-4` |

Hydro: dry mass **19 kg**. Side floats from `HAUV/Float - Part 1.stl` → `float.stl` at y=±0.45 (STEP float BREP was empty).

```sh
cd AUAV-sim-ros/sim/cad
.venv/bin/python prepare_propulsion_meshes.py
../../gz_standalone/px4_custom_setup.sh
```

## Frame

STEP/STL millimeters → meters. Gazebo FLU: +X forward, +Z up.

Airframe meshes are **yawed +90°** after export so pontoon long axes align with
`+X`, CAD nose = `+X`, and the T200 sits on the aft (`−X`) face with thrust
along `+X`. See `frame_align.json`. Backups: `hauv_big_pre_yaw.stl`,
`hauv_big_collision_pre_yaw.stl`.
