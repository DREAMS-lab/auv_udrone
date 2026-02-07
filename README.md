# AUV UDrone

Custom PX4 airframe and Gazebo SITL simulation for a torpedo-shaped autonomous
underwater vehicle (AUV) with Blue Robotics T200 thrusters.

## Vehicle Specifications

| Property | Value |
|----------|-------|
| Hull length | 17 in (0.432 m) |
| Hull diameter | 10 in (0.254 m) |
| Total mass | 13 kg |
| Thrusters | 4 × Blue Robotics T200 (0.344 kg each) |
| Battery | 12 V 30 Ah LiFePO4 (4S) |
| Airframe ID | 60003 |
| MAV_TYPE | 12 (Submarine) |

### Thruster Layout

Four T200 thrusters mounted at the rear of the hull in a square pattern, all
thrusting forward along the X-axis. Differential thrust provides roll, pitch,
and yaw authority.

| # | Position (m) | Label |
|---|-------------|-------|
| 0 | (−0.26, −0.15, −0.15) | starboard-bottom |
| 1 | (−0.26, +0.15, −0.15) | port-bottom |
| 2 | (−0.26, +0.15, +0.15) | port-top |
| 3 | (−0.26, −0.15, +0.15) | starboard-top |

### Hydrodynamics

The Gazebo model includes Fossen's 6-DOF hydrodynamics (`gz-sim-hydrodynamics-system`)
with added mass and drag coefficients derived from the hull geometry:

- **Added mass**: prolate-spheroid Lamb k-factors (a/b ≈ 1.7)
- **Quadratic drag**: form drag from hull frontal area (Cd=0.15) and broadside
  area (Cd=1.1) including T200 thruster frames
- **Linear drag**: viscous skin-friction for low-speed regime

## Repository Structure

```
airframes/
  60003_auv_udrone         # Hardware airframe (Pixhawk Cube 2.1)
  60003_gz_xudrone         # Gazebo SITL airframe
simulation/
  models/xudrone/
    model.config           # Gazebo model metadata
    model.sdf              # Gazebo SDF (hull, thrusters, sensors, hydro)
scripts/
  rc_act_controls.py       # RC → MAVLink actuator controls
  testuDroneNanoKontrol.py # NanoKontrol test setpoints
```

## Installation

These files are designed to be overlaid onto a [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) checkout.

```bash
# Clone PX4-Autopilot (if not already done)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Clone this repo
git clone https://github.com/DREAMS-lab/auv_udrone.git /tmp/auv_udrone

# Copy airframe files
cp /tmp/auv_udrone/airframes/60003_auv_udrone \
   ROMFS/px4fmu_common/init.d/airframes/
cp /tmp/auv_udrone/airframes/60003_gz_xudrone \
   ROMFS/px4fmu_common/init.d-posix/airframes/

# Copy Gazebo model
cp -r /tmp/auv_udrone/simulation/models/xudrone \
   Tools/simulation/gz/models/
```

Then register the airframes in the PX4 CMakeLists:

**`ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt`** — add inside
the `CONFIG_MODULES_UUV_ATT_CONTROL` block:
```cmake
60003_auv_udrone
```

**`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`** — add
after `60002_gz_uuv_bluerov2_heavy`:
```cmake
60003_gz_xudrone
```

## Running SITL

```bash
# Clean params (first time or after config changes)
rm -rf build/px4_sitl_default/tmp/rootfs/eeprom/parameters*

# Launch Gazebo SITL
make px4_sitl gz_xudrone
```

This starts PX4 with the `underwater` Gazebo world and spawns the xudrone model.

## License

Apache-2.0
