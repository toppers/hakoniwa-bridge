English ｜ [日本語](README-ja.md)

# What is this?

This is a simple sample code for the Hakoniwa Bridge.

With this sample code, you can test the following:

- Writing PDU data from a virtual Hakoniwa asset allows it to be referenced on the edge side.
- Delivering PDU data from the edge side allows it to be referenced by the virtual Hakoniwa asset.

# Configuration

The configuration of the sample program is as shown in the diagram below.

![example](../images/example.png)

The virtual side Hakoniwa asset, HakoAssetSample, performs read and write operations on the Hakoniwa PDU data.

The data read by HakoAssetSample is `RobotAvator_cmd_pos`.

The data written by HakoAssetSample are `RobotAvator_baggage_sensor` and `RobotAvator_bumper_sensor`.

These data are shared with the edge side (Real) through ShmProxy.

On the real side, it can be published/subscribed as ROS topics through RosProxy.

The data written by HakoAssetSample, `RobotAvator_baggage_sensor` and `RobotAvator_bumper_sensor`, can be referenced using ros2 topic echo.

The data read by HakoAssetSample, `RobotAvator_cmd_pos`, can be sent using the ros2 topic echo command.

# Preparation

Please install the necessary software on both the virtual and edge sides beforehand.

Build the sample program for the virtual side.

```
cd examples
```

```
bash build.bash
```

Upon success, the following file will be created:

```
% ls cmake-build/sample
cmake-build/sample
```

# How to Run

## On the Virtual Side

Start ShmProxy.

```
./cmake-build/shm-proxy/shm-proxy ShmProxy ../third-party/hakoniwa-ros2pdu/config/custom.json 20 master
```

Start the sample program.

```
./cmake-build/sample HakoSampleAsset ./custom.json 20
```

Upon success, logs will be output every second as follows:

```
Robot: RobotAvator, PduWriter: RobotAvator_baggage_sensor
channel_id: 1 pdu_size: 4
INFO: RobotAvator create_lchannel: logical_id=1 real_id=1 size=4
Robot: RobotAvator, PduWriter: RobotAvator_bumper_sensor
channel_id: 2 pdu_size: 4
INFO: RobotAvator create_lchannel: logical_id=2 real_id=2 size=4
INFO: asset(HakoSampleAsset) is registered.
WAIT START
WAIT RUNNING
PDU CREATED
LOADED: PDU DATA
INFO: my_on_initialize enter
INFO: sleep 1sec
INFO: my_on_initialize exit
INFO: start simulation
SYNC MODE: true
INFO: on_simulation_step enter: 20000
20000: pos data(0.000000, 0.000000, 0.000000)
INFO: on_simulation_step exit
INFO: on_simulation_step enter: 40000
40000: pos data(0.000000, 0.000000, 0.000000)
INFO: on_simulation_step exit
INFO: on_simulation_step enter: 60000
60000: pos data(0.000000, 0.000000, 0.000000)
INFO: on_simulation_step exit
```

Start the simulation with the Hakoniwa command.

```
hako-cmd start
```

## On the Edge Side

Start RosProxy.

```
ros2 run hako_ros_proxy hako_ros_proxy_node 
```

Display the list of ROS2 topics.

```
ros2 topic list
/RobotAvator_baggage_sensor
/RobotAvator_bumper_sensor
/RobotAvator_cmd_pos
/parameter_events
/rosout
```

Reference the data written by HakoAssetSample, `RobotAvator_baggage_sensor` and `RobotAvator_bumper_sensor`, using ros2 topic echo.

RobotAvator_baggage_sensor:
```
ros2 topic echo /RobotAvator_baggage_sensor
```
Output: Display continues toggling between true and false
```
$ ros2 topic echo /RobotAvator_baggage_sensor
data: true
---
data: false
---
data: true
---
data: false
```

RobotAvator_bumper_sensor:
```
ros2 topic echo /RobotAvator_bumper_sensor:
```

Output: Display continues toggling between true and false
```
$ ros2 topic echo RobotAvator_bumper_sensor
data: true
---
data: false
---
data: true
---
data: false
```

Send the data read by HakoAssetSample, `RobotAvator_cmd_pos`, using the ros2 topic echo command as follows.

```
ros2 topic pub RobotAvator_cmd_pos geometry_msgs/msg/Twist

 "{linear: {x: 3.5, y: 2.0, z: 1.0}, angular: {x: 0.1, y: 0.1, z: 0.2}}"
```

Upon success, the virtual side will display the received data as follows:

```
INFO: on_simulation_step enter: 2500000
2500000: pos data(3.500000, 2.000000, 1.000000)
INFO: on_simulation_step exit
```
