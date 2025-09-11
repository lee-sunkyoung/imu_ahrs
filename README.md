# path_planner

This package provides filtering for Attitude, Heading, and Reference System (AHRS) using IMU data.

It implements:

- **Mahony's algorithm**
- **Slerp (spherical linear interpolation)**

 for smooth quaternion filtering

Additionally, the current IMU orientation can be **visualized in RViz2**.

- [Topics](#Topics)
- [Prerequisites](#Prerequisites)
- [How to Use](#how-to-use)

## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill     |
| **Eigen3**  | 3.4.0            |
| **IMU tested**  | Xsens MTi-610      |

## Topics
### Input

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/acceleration` | `geometry_msgs/msg/Vector3` | Accelerometer data |
| `/imu/angular_velocity` | `geometry_msgs/msg/Vector3` | Gyroscope data |
| `/imu/mag` | `geometry_msgs/msg/Vector3` | Magnetometer data |

### Output

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/msg/Imu` | Filtered IMU data |
| `/pose_data` | `geometry_msgs/msg/PoseStamped` | Filtered pose (frame_id: `pose`) |

## Prerequisites

1) You need `/imu/acceleration`,`/imu/angular_velocity`,`/imu/mag` data to use this package.

2) Make sure to install the required package:
```
sudo apt update
sudo apt install libeigen3-dev
```

## How to Use
### 1. Build & Run
1) edit `mti_imu/config/param.yaml` to match your environment. 

2) Build & Run
```
cd <your_ws> && colcon build
ros2 run mti_imu mti_imu
```

### 2. Visualize imu position in RViz2:

1) to use visualization, run command below.

```
ros2 launch mti_imu mti_imu_launch.py
```

2) run rviz2, set Fixed Frame `pose`, add topic `/pose_data` set shape `Axes`.


## more info
- [Mahony R, Hamel T, Pflimlin J M. Nonlinear complementary filters on the special orthogonal group[J]. IEEE Transactions on automatic control, 2008, 53(5): 1203-1218.](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4608934)  

- [Xsens ROS2 driver](https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/ros2)

- [Xsens MTi software-documentation](https://www.movella.com/support/software-documentation)


