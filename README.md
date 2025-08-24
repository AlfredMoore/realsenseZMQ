# realsenseZMQ

This repo is built to send RealSense frames through ZMQ.

The ROS based Image Communication is too slow and requires higher bandwidth.

## How to use
```bash
mkdir build && cd build
cmake ..
cmake --build . -j 16   # make -j16
```
Compile

```bash
./rs_zmq_publisher --serial <RealsenseSerial>
```

## Integrate in ROS2
[Example](https://github.com/AlfredMoore/stretch_ros2/tree/humble/external_dev)
