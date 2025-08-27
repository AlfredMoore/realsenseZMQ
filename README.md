# realsenseZMQ

This repo is built to send RealSense frames through ZMQ.

The ROS based Image Communication is too slow and requires higher bandwidth. Default using `tcp://*:5555`.

This is only the image publisher on robot. The ZeroMQ subscriber should run on XR device or other computational device.

## How to use
```bash
mkdir build && cd build
cmake ..
cmake --build . -j 16   # make -j16
```
Compile

```bash
./rs_zmq_publisher
```
Raw scan Realsense camera. You can get your Realsense serial here.

```bash
./rs_zmq_publisher --serial <RealsenseSerial>
```
Run with your desired serial number.

```bash
./rs_zmq_publisher --serial <RealsenseSerial> --show
```
Run and show in local screen.

## Integrate in ROS2
[Example](https://github.com/AlfredMoore/stretch_ros2/tree/humble/external_dev)
