# AUV uDrone (CoRAL Robotic Vehicles)

[![GitHub](https://img.shields.io/badge/GitHub-DREAMS--lab%2Fauv__udrone-blue?logo=github)](https://github.com/DREAMS-lab/auv_udrone)

PX4 airframe definitions and Gazebo Harmonic SITL simulations for the
**uDrone** AUV and **Roboato** ASV, developed at ASU
[DREAMS Lab](https://search.asu.edu/profile/2aborning) as part of the
[CoRAL (Collaborative Robotic Aquatic Laboratory)](https://github.com/Earth-Innovation-Hub/CoRAL)
project.

## Project Context

The CoRAL system enables collaborative multi-robot monitoring of coral reef
ecosystems. The two primary marine vehicles are:

- **uDrone** — a torpedo-shaped AUV for terrain-following and semantic mapping
  of coral reefs at depth
- **Roboato** (Robo-boat-o) — a Heron-class catamaran ASV that serves as a
  surface hub, deploying/recovering the uDrone and carrying out vertical water
  column profiling with a winched sonde

The vehicles operate in tandem (tethered or untethered), communicating via
MAVLink and networked through ROS. The
[udrone_boat_localization](https://github.com/DREAMS-lab/udrone_boat_localization)
package provides EKF/UKF state estimators for the joint uDrone-boat system.

### Onboard Systems

| System | uDrone (AUV) | Roboato (ASV) |
|--------|-------------|---------------|
| Flight controller | Pixhawk Cube 2.1 / PX4 | Pixhawk 4 / PX4 |
| Compute | NUC i7 + NCS, or Jetson TX2 | Intel NUC i7 |
| Vision | RealSense T265 + RGB HD | — |
| Battery | 12 V 30 Ah LiFePO4 (4S) | 4S LiFePO4 |
| Propulsion | 4 × Blue Robotics T200 | 2 × water jet (differential) |

### Field Deployments

- [2020 uDrone pool test](https://www.youtube.com/watch?v=sM6XzXgjIlo)
- [2022 uDrone field test](https://www.youtube.com/watch?v=zxxR6npl8X8&t=2s)
- [2023 uDrone onboard cameras](https://youtu.be/JJ8X70HSR94)
- [2023 uDrone external footage](https://www.youtube.com/shorts/YMnUrwg8ljE)
- [Roboato test at The Lakes](https://www.youtube.com/watch?v=Wey2bst0cBY)

---

## uDrone (AUV)

| Property | Value |
|----------|-------|
| Hull | 17 in (0.432 m) long, 10 in (0.254 m) diameter |
| Total mass | 13 kg |
| Thrusters | 4 × Blue Robotics T200 (rear corners, all forward) |
| Battery | 12 V 30 Ah LiFePO4 (4S) |
| PX4 Airframe ID | 60003 |
| MAV_TYPE | 12 (Submarine) |
| SITL command | `make px4_sitl gz_xudrone` |

Four T200 thrusters at the rear in a square pattern provide roll, pitch, and
yaw authority through differential thrust. The Gazebo model includes Fossen
6-DOF hydrodynamics, VIO odometry publisher, and full sensor suite.

| # | Position (m) | Label |
|---|-------------|-------|
| 0 | (−0.26, −0.15, −0.15) | starboard-bottom |
| 1 | (−0.26, +0.15, −0.15) | port-bottom |
| 2 | (−0.26, +0.15, +0.15) | port-top |
| 3 | (−0.26, −0.15, +0.15) | starboard-top |

---

## Roboato (ASV)

| Property | Value |
|----------|-------|
| Hull | Catamaran, 1.35 m long, 0.98 m wide, 0.32 m tall |
| Total mass | 28 kg |
| Thrusters | 2 × water jet (differential drive, rear of each pontoon) |
| Draft | 0.15 m |
| Max speed | ~1.7 m/s (3.3 knots) |
| PX4 Airframe ID | 55000 |
| MAV_TYPE | 11 (Surface Boat) |
| SITL command | `make px4_sitl gz_xroboato` |

Based on the [Clearpath Heron](https://clearpathrobotics.com) catamaran design.
Two pontoons (0.70 m center-to-center) with differential water jet thrusters
enable zero-radius turning. The Gazebo model includes Fossen 6-DOF
hydrodynamics tuned for a wide-beam catamaran (high sway/yaw drag).

| # | Position (m) | Label |
|---|-------------|-------|
| 0 | (−0.50, −0.49) | port thruster |
| 1 | (−0.50, +0.49) | starboard thruster |

---

## Repository Structure

```
airframes/
  60003_auv_udrone         # uDrone hardware airframe
  60003_gz_xudrone         # uDrone Gazebo SITL airframe
  55000_asv_roboato        # Roboato hardware airframe
  55000_gz_xroboato        # Roboato Gazebo SITL airframe
simulation/
  models/xudrone/
    model.config
    model.sdf              # AUV: torpedo hull, T200s, hydro, VIO
  models/xroboato/
    model.config
    model.sdf              # ASV: catamaran hull, water jets, hydro
scripts/
  rc_act_controls.py       # RC -> MAVLink actuator controls
  testuDroneNanoKontrol.py # NanoKontrol test setpoints
```

## Installation

Overlay these files onto a
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) checkout:

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

git clone https://github.com/DREAMS-lab/auv_udrone.git /tmp/auv_udrone

# uDrone airframes
cp /tmp/auv_udrone/airframes/60003_auv_udrone \
   ROMFS/px4fmu_common/init.d/airframes/
cp /tmp/auv_udrone/airframes/60003_gz_xudrone \
   ROMFS/px4fmu_common/init.d-posix/airframes/
cp -r /tmp/auv_udrone/simulation/models/xudrone \
   Tools/simulation/gz/models/

# Roboato airframes
cp /tmp/auv_udrone/airframes/55000_asv_roboato \
   ROMFS/px4fmu_common/init.d/airframes/
cp /tmp/auv_udrone/airframes/55000_gz_xroboato \
   ROMFS/px4fmu_common/init.d-posix/airframes/
cp -r /tmp/auv_udrone/simulation/models/xroboato \
   Tools/simulation/gz/models/
```

Then register the airframes in PX4 CMakeLists:

**`ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt`**:
```cmake
# In CONFIG_MODULES_UUV_ATT_CONTROL block:
60003_auv_udrone

# In CONFIG_MODULES_ROVER_DIFFERENTIAL block:
55000_asv_roboato
```

**`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`**:
```cmake
60003_gz_xudrone
55000_gz_xroboato
```

## Running SITL

```bash
# Clean params (first time or after config changes)
rm -rf build/px4_sitl_default/tmp/rootfs/eeprom/parameters*

# Launch uDrone (underwater world)
make px4_sitl gz_xudrone

# Launch Roboato (default world)
make px4_sitl gz_xroboato
```

## Related Projects

- [CoRAL — Collaborative Robotic Aquatic Laboratory](https://github.com/Earth-Innovation-Hub/CoRAL)
- [udrone_boat_localization](https://github.com/DREAMS-lab/udrone_boat_localization) — EKF/UKF state estimators for the uDrone-boat tandem system
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

## Acknowledgements

- NSF CNS 1521617
- NASA Flight Opportunities Tech Flights Lunar Lander ExoCam
- Earth Innovation Hub

## License

Apache-2.0
