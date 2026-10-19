# HAUV notes (firmware tree)

Gazebo models/worlds follow the **vflydrone `gz_standalone` pattern**:

1. **Canonical:** [`AUAV-sim-ros/gz_standalone/`](../../../AUAV-sim-ros/gz_standalone)
2. **Installed copy:** [`Tools/hauv/gz/`](gz/) — filled by `px4_custom_setup.sh`
   for CMake `GLOB` (`gz_hauv_4plus1_*`) and native fallback

Edit assets in `gz_standalone`, then inject (or let Docker entrypoint do it):

```sh
cd ../../../AUAV-sim-ros/gz_standalone
PX4_DIR=$PWD/../../PX4-Autopilot-VinUni-AUAV ./px4_custom_setup.sh
```

Also in this PX4 tree:

- `src/modules/hauv_water_control/` — Water-mode controller
- `ROMFS/.../4902_gz_hauv_4plus1` — SITL airframe / actuator allocation
- `gz_bridge` — prefers `HAUV_GZ_ROOT`, falls back to `Tools/hauv/gz`

## Run

```sh
cd ../../../AUAV-sim-ros/sim
./scripts/test_gz_model.sh
./scripts/build_px4.sh
./scripts/up.sh
SIM_MODE=water ./scripts/up.sh
```

CAD / propulsion mesh refresh:

```sh
cd ../../../AUAV-sim-ros/sim/cad
./export_big.sh
.venv/bin/python prepare_propulsion_meshes.py
cd ../../gz_standalone && PX4_DIR=$PWD/../../PX4-Autopilot-VinUni-AUAV ./px4_custom_setup.sh
```

See the main guide: [`AUAV-sim-ros/README.md`](../../../AUAV-sim-ros/README.md).
