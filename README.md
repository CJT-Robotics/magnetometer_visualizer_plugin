# Magnetometer Visualizer Plugin (RQT)

## Overview

The **Magnetometer Visualizer Plugin** is an RQT plugin for ROS1 that provides visualization and monitoring of magnetic field data published as `sensor_msgs/MagneticField` messages.

The plugin displays a live graph of X, Y, and Z magnetic field components and provides configurable thresholds and scaling.

**Works perfectly with our [magnetometer package](https://github.com/CJT-Robotics/magnetometer)**

## Requirements
Install required ROS packages:

```bash
sudo apt install ros-noetic-rqt ros-noetic-rqt-common-plugins
```

Make sure the following Python packages are installed:

```bash
pip3 install numpy matplotlib
```

## Installation

Clone the repository into your Catkin workspace:

```bash
cd ~/catkin_ws/src
git clone git@github.com:CJT-Robotics/magnetometer_visualizer_plugin.git
```

Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
```

Source your workspace:

```bash
source devel/setup.bash
```

## Usage

Run your node that publishes `sensor_msgs/MagneticField`.

Launch RQT:

```bash
rqt --force-discover # After first run, --force-discover is unnecessary
```

Load the plugin:
Plugins → CJT-Robotics → Magnetic Field Monitor

## Configuration

### Topic Selection

* Automatically lists all topics of type `sensor_msgs/MagneticField`
* Or type in manually a topic name

### Settings

* **Unit Scaling**: Convert Tesla values to mT, µT, or nT
* **Buffer**: Number of stored data points (history length)
* **Threshold**: Detection threshold for highlighting anomalies
* **Y-Axis**: Controls zoom level of the graph

### Visualization

* Red: X-axis
* Green: Y-axis
* Blue: Z-axis
* Yellow highlight: Values exceeding threshold
* Status box:

  * `MONITORING`: Normal operation
  * `DETECTION`: Threshold exceeded

## Troubleshooting

### Plugin not visible in RQT

* Make sure the workspace is sourced:

```bash
source ~/catkin_ws/devel/setup.bash
```

### No topics available

* Ensure a node is publishing `sensor_msgs/MagneticField`
* Check with:

```bash
rostopic list
```

### No data shown

* Verify message content:

```bash
rostopic echo /your_topic
```