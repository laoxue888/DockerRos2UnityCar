
---

[TOC]

# 前言

unity和ros2的通信测试。

# 测试

```shell
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.launch.py
```

```shell
ros2 topic list
```

```shell
ros2 topic echo /demo_talker
```

```shell
ros2 topic echo /image_talker
```

```shell
source install/setup.bash
ros2 run unity_control_example unity_control_node
```