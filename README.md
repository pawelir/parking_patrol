# parking_patrol

This package functionality is to detect free spots on the parking site. Provides API for retrieving coordinates of detected free parking places.

## Building from Source

[![Source build](https://github.com/pawelir/parking_patrol/actions/workflows/colcon-test-build.yaml/badge.svg)](https://github.com/pawelir/parking_patrol/actions/workflows/colcon-test-build.yaml)

### Dependencies

External dependencies:

* `geometry_msgs`
* `nav_msgs`
* `sensor_msgs`
* `std_srvs`

### Building

To build from source, clone the latest version from the repository into your workspace and compile the package using:

```bash
cd workspace/src
git clone https://github.com/pawelir/parking_patrol.git
cd ../
rosdep update
rosdep install -i --from-paths src -y
colcon build --symlink-install
```

_**Note:** ROS should be sourced before: `source /opt/ros/<your-distro>/setup.bash`_

## Usage

Launch the core functionality with:

```bash
ros2 launch parking_patrol parking_patrol.launch.py
```

### Config files

* **parking_patrol.yaml** - config file allowing the node parameters settings

### Launch files

* **parking_patrol.launch.py** - main launch file launching the whole functionality

## Nodes

### parking_patrol_node

High level node providing API for restarting detection process and retrieving the data after scanning.

#### Service servers

* **`~/restart_patrol`** _std_srvs/Trigger_ - Trigger cleaning detection buffer

  ```bash
  ros2 service call /parking_patrol_node/restart_patrol std_srvs/srv/Trigger "{}"
  ```

* **`~/finish_patrol`** _parking_patrol_msgs/FreeSpots_ - Get detected parking spots

  ```bash
  ros2 service call /parking_patrol_node/finish_patrol parking_patrol_msgs/srv/FreeSpots "{}"
  ```

#### Parameters

| Name                       | Type  | Default value | Description                                                                              |
| -------------------------- | ----- | ------------- | ---------------------------------------------------------------------------------------- |
| `lidar_angular_resolution` | float | 1.0           | LIDAR angular resolution - depends on your simulation settings or hardware specification |
| `spot_depth`               | float | 0.7           | Spot characteristics - spot depth [m]                                                    |
| `spot_width`               | float | 0.4           | Spot characteristics - spot width [m]                                                    |

### spot_finder_node

Node responsible only for detecting free spots in the neighborhood.

#### Subscribed Topics

* **`/odom`** _nav_msgs/msg/Odometry_
* **`/scan`** _sensor_msgs/msg/LaserScan_
* **`/imu`** _sensor_msgs/msg/Imu_
