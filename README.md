# Parking Patrol

---

## Synopsis

Parking patrol provides functionality to detect free spots on parking sites.

## Highlights

* Parking patrol allows specifying the parking details: it can be read from
  the `parking_patrol.yaml`.

---

## Installation and running

### Building

To build and install project proceed with ROS2 **colcon** command:

```bash
colcon build --symlink-install
```

For unit testing, execute it with CMake arguments:

```bash
 colcon build --cmake-args -DBUILD_TESTING=ON
```

**Note:** Remember to source the project (`source ./install/local_setup.zsh`) before running!


---
### Launch Parking Patrol

It is possible to use the following command in order to launch two nodes responsible for spot detection:

```bash
ros2 launch parking_patrol parking_patrol.launch.py
```

---

## ROS2 interaction from CLI examples

### Services

* `/parking_patrol_node/finish_patrol`

```bash
ros2 service call /parking_patrol_node/finish_patrol parking_patrol_msgs/srv/FreeSpots {}

```

* `/parking_patrol_node/restart_patrol`

```bash
ros2 service call /parking_patrol_node/restart_patrol std_srvs/srv/Trigger {}
```

# Developer notes

***Notice:*** All ideas and bugs should be reported through GitHub Issues.

#### **automation_node dependencies**

| Package        | Node      | Path                                       | Type           | Definition                                  |
| -------------- | --------- | ------------------------------------------ | -------------- | ------------------------------------------- |
| ParkingPatrol  | spot_finder_node | `/imu`                              | Subscriber     | Imu msg                               |
| ParkingPatrol  | spot_finder_node | `/odom`                             | Subscriber     | Odom msg                               |
| ParkingPatrol  | spot_finder_node | `/laser/scan`                       | Subscriber     | LaserScan msg                               |
