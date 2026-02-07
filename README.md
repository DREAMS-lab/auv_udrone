# AUV uDrone

[![GitHub](https://img.shields.io/badge/GitHub-DREAMS--lab%2Fauv__udrone-blue?logo=github)](https://github.com/DREAMS-lab/auv_udrone)

PX4 airframe definition and Gazebo Harmonic SITL simulation for the **uDrone**,
an autonomous underwater vehicle developed at ASU
[DREAMS Lab](https://search.asu.edu/profile/2aborning) as part of the
[CoRAL (Collaborative Robotic Aquatic Laboratory)](https://github.com/Earth-Innovation-Hub/CoRAL)
project.

## Project Context

uDrone is a custom AUV designed for terrain-following and semantic mapping of
coral reef systems. It operates as part of a collaborative multi-robot team
alongside the *Robo-boat-o* autonomous surface vessel and aerial imaging
systems, enabling high-resolution data collection across the water column from
100 m depth to 100 m altitude.

The vehicle can operate tethered or untethered in tandem with the robotic boat.
Tethered mode provides tighter communication for testing new science autonomy
algorithms and GNC tuning, while untethered operation is used for field
experiments. The system communicates using MAVLink protocol and is networked
using ROS.

### Onboard Systems

| System | Hardware |
|--------|----------|
| Flight controller | Pixhawk Cube 2.1 running PX4 |
| Compute | Intel NUC i7 + Neural Compute Stick, or Jetson TX2 + Celeron x86 |
| Vision | Intel RealSense T265 (stereo fisheye) + RGB HD camera |
| Battery | 12 V 30 Ah LiFePO4 (4S) |
| Thrusters | 4 × Blue Robotics T200 |

The vision package enables visual-inertial odometry (VIO) and semantic SLAM for
autonomous terrain-relative reef mapping. Stacked controllers provide position,
velocity, attitude, and attitude-rate control.

### Field Deployments

The vehicle has been developed and tested at pool facilities, and deployed at
the Bermuda Institute of Ocean Sciences (BIOS). Testing is documented in video:

- [2020 pool test](https://www.youtube.com/watch?v=sM6XzXgjIlo)
- [2022 field test](https://www.youtube.com/watch?v=zxxR6npl8X8&t=2s)
- [2023 onboard cameras](https://youtu.be/JJ8X70HSR94)
- [2023 external footage](https://www.youtube.com/shorts/YMnUrwg8ljE)

## Vehicle Specifications

| Property | Value |
|----------|-------|
| Hull length | 17 in (0.432 m) |
| Hull diameter | 10 in (0.254 m) |
| Total mass | 13 kg |
| Thrusters | 4 × Blue Robotics T200 (0.344 kg each) |
| Battery | 12 V 30 Ah LiFePO4 (4S) |
| PX4 Airframe ID | 60003 |
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

### Sensors (Gazebo SITL)

| Sensor | Source | EKF2 Fusion |
|--------|--------|-------------|
| IMU | Gazebo `imu_sensor` | Always |
| Magnetometer | Gazebo `magnetometer_sensor` + MAGSIM | Heading |
| Barometer | `SENS_EN_BAROSIM` | Depth/pressure |
| GPS | `SENS_EN_GPSSIM` | Disabled (`EKF2_GPS_CTRL=0`) |
| External Vision (VIO) | Gazebo `OdometryPublisher` | Position + yaw (`EKF2_EV_CTRL=15`) |

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
  rc_act_controls.py       # RC -> MAVLink actuator controls
  testuDroneNanoKontrol.py # NanoKontrol test setpoints
```

## Installation

These files are designed to be overlaid onto a
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) checkout.

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

## Related Projects

- [CoRAL — Collaborative Robotic Aquatic Laboratory](https://github.com/Earth-Innovation-Hub/CoRAL)
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

## Acknowledgements

- NSF CNS 1521617
- NASA Flight Opportunities Tech Flights Lunar Lander ExoCam
- Earth Innovation Hub

## License

Apache-2.0
