# HAUV CAD meshes

## Airframe

From `HAUV/AUAV-draft.step` / `HAUV-v2.step` product **Big** → **COMPOUND010**.
SunnySky motor assemblies are excluded; their centers set Gazebo rotor poses.

| File | Role |
|---|---|
| `hauv_big.stl` | Visual mesh (meters) |
| `hauv_big_collision.stl` | Convex-hull collision mesh |
| `placements.json` | BBox + rotor / T500 pose hints |

```sh
cd AUAV-sim-ros/sim/cad && ./export_big.sh
```

## Propulsion (18×6.1 + T500)

Source STL from Shapr (not embedded in STEP — empty placeholders only):

- `HAUV/18x6.1.STL`
- `HAUV/T500-THRUSTER-R1.STL`

| File | Role |
|---|---|
| `prop_18x6_1_ccw.stl` / `prop_18x6_1_cw.stl` | Aerial props (spin about +Z) |
| `motor_sunnysky.stl` | SunnySky V5210 (×4 on `base_link`) |
| `t500_thruster.stl` | T500 body (aft bulkhead, fixed on `base_link`) |
| `propulsion_meshes.json` | Extents / regenerate metadata |
| `propulsion_physics.json` | Datasheet → Gazebo `motorConstant` / `maxRotVelocity` |

Physics (Gazebo `thrust = motorConstant · ω²`):

| Actuator | Source | ω_max | T_max | motorConstant |
|---|---|---|---|---|
| Aerial ×4 | V5210 + C18 @ 25 V | 603 rad/s | ~36 N | `9.80e-5` |
| T500 | Blue Robotics @ 16 V | 246 rad/s | ~91 N | `1.51e-3` |

```sh
cd AUAV-sim-ros/sim/cad
.venv/bin/python prepare_propulsion_meshes.py
# motor from STEP (FreeCAD Docker):
#   HAUV_STEP_PATH=../../HAUV-v2.step HAUV_OUT_STL=.../motor_sunnysky.stl \
#   docker ... export_motor_from_step.py
../../gz_standalone/px4_custom_setup.sh
```

## Frame

STEP/STL millimeters → meters. Gazebo FLU: +X forward, +Z up.

Airframe meshes are **yawed +90°** after export so pontoon long axes align with
`+X`, CAD nose = `+X`, and the T500 sits on the aft (`−X`) face with thrust
along `+X`. See `frame_align.json`. Backups: `hauv_big_pre_yaw.stl`,
`hauv_big_collision_pre_yaw.stl`.
