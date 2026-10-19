# HAUV CAD meshes

## Airframe

From `HAUV/AUAV-draft.step` / `HAUV-v2.step` product **Big** → **COMPOUND010**.
SunnySky assemblies are excluded from the visual; their centers set Gazebo rotor
poses.

| File | Role |
|---|---|
| `hauv_big.stl` | Visual mesh (meters), yawed +90° into Gazebo FLU |
| `hauv_big_collision.stl` | Convex-hull collision (not used for buoyancy volume) |
| `placements.json` | BBox + rotor / T500 pose hints from STEP |
| `frame_align.json` | Baked yaw metadata + current T500 / rotor poses |

```sh
cd AUAV-sim-ros/sim/cad && ./export_big.sh
```

## Propulsion (18×6.1 + T500)

Source STLs from Shapr (not embedded as usable solids in STEP):

- `HAUV/18x6.1.STL`
- `HAUV/T500-THRUSTER-R1.STL`

| File | Role |
|---|---|
| `prop_18x6_1_ccw.stl` / `prop_18x6_1_cw.stl` | Aerial props (spin about +Z) |
| `motor_sunnysky.stl` | SunnySky V5210 (×4 fixed on `base_link`) |
| `t500_thruster.stl` | T500 housing on `base_link` (center aft bulkhead hole) |
| `propulsion_meshes.json` | Extents / regenerate metadata |

```sh
cd AUAV-sim-ros/sim/cad
.venv/bin/python prepare_propulsion_meshes.py
../../gz_standalone/px4_custom_setup.sh
```

## Frame

- STEP/STL: millimeters → meters.
- Gazebo FLU: `+X` forward, `+Y` left, `+Z` up.
- Airframe meshes are **yawed +90°** after export so pontoon long axes align
  with `+X` and CAD nose = `+X`. T500 mounts in the **center aft bulkhead**
  opening with thrust along `+X` (mesh local `+Z` → body `+X` via pitch `π/2`).
- Buoyancy uses invisible box collisions in `model.sdf` (visual size unchanged).
- Native-frame backups `*_pre_yaw.stl` are gitignored; regenerate with the CAD
  scripts if you need to re-bake yaw.
