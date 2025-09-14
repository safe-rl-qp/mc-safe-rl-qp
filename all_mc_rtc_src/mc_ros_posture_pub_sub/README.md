# RosPosturePubSub

**RosPosturePubSub** is a plugin for the [mc\_rtc](https://jrl-umi3218.github.io/mc_rtc/) control framework that enables real-time robot posture state communication between an mc\_rtc robot controller and ROS 2 via the `trajectory_msgs/msg/JointTrajectory` and `control_msgs/msg/JointTrajectoryControllerState` messages.

This plugin allows:

* Subscribing to a posture command from ROS 2, which is forwarded to an mc\_rtc controller.
* Publishing the current posture (joint positions and velocities) of the mc\_rtc robot to ROS 2 for external monitoring or feedback control.

---

## Features

* Subscribe to joint posture trajectories over ROS 2.
* Publish current joint states from the mc\_rtc controller.
* Seamlessly integrates with ROS 2 controllers (e.g., `ros2_control`, MoveIt).
* Uses mc\_rtc `datastore` for data exchange between the plugin and the controllers.

## How It Works

### Data Flow

The message types are fixed unless the code is modified, but the topic names are configurable via the plugin's configuration file. The default topics are:

|          Direction | ROS 2 Topic                                     | Message Type                                      |
| -----------------: | ----------------------------------------------- | ------------------------------------------------- |
| mc\_rtc subscriber | `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory`             |
|  mc\_rtc publisher | `/joint_trajectory_controller/state`            | `control_msgs/msg/JointTrajectoryControllerState` |

### mc\_rtc `datastore`

* The plugin writes the received posture into:

  ```cpp
  ctl.controller().datastore()["ros_posture_pub_sub"]
  ```

  as a `std::map<std::string, std::vector<double>>` for downstream access.

  The output stored under the "ros\_posture\_pub\_sub" key in the datastore can be directly used as a target for an mc\_rtc posture task.

---

## Configuration Example

```yaml
ros_posture_pub_sub:
  command_topic: "/joint_trajectory_controller/joint_trajectory"
  state_topic: "/joint_trajectory_controller/state"
```
