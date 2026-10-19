# HAUV CAD meshes

Generated from `HAUV/AUAV-draft.step` product **Big** → child **COMPOUND010**
(10 airframe solids). Nested SunnySky motor STEP assemblies are excluded from
the visual mesh; their centers are used for Gazebo rotor poses.

These meshes live under **AUAV-sim-ros/gz_standalone** (injected into PX4 via
`px4_custom_setup.sh`).

| File | Role |
|---|---|
| `hauv_big.stl` | Visual mesh (meters) |
| `hauv_big_collision.stl` | Convex-hull collision mesh |
| `placements.json` | BBox + suggested rotor / T500 poses |

## Regenerate

```sh
cd AUAV-sim-ros/sim/cad
./export_big.sh
```

Requires Docker image `amrit3701/freecad-cli` and the local venv under
`sim/cad/.venv` (created by the first export for convex-hull post-process).

## Frame

STEP units are treated as millimeters and scaled to meters. Gazebo FLU:
+X forward, +Z up. If the mesh appears rotated in sim, adjust the visual /
collision `<pose>` RPY in `model.sdf` and record the transform here.
