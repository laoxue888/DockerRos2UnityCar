
---

## 前言

本项目介绍一种基于**Unity3D**引擎的**SLAM**多传感器仿真平台，可模拟 **IMU、RGB-D相机、激光雷达**传感器数据输出，为SLAM算法提供**低成本、高可控性、可复现**的仿真测试环境。

![alt text](images/rgbd_lidar.gif)

video:
> - [2025-6-19：【开源】SLAM技术之Unity3D仿真激光雷达、RGBD相机、IMU传感器](https://www.bilibili.com/video/BV1twNxz1E7s/?vd_source=3bf4271e80f39cfee030114782480463)
> - [2025-4-1:【开源】实现在unity3d中仿真小车，还能用机器人开发框架ros2控制喔](https://www.bilibili.com/video/BV1CGZbY6ESv/?vd_source=3bf4271e80f39cfee030114782480463)

## 搭建开发环境

> - Unity:2023
> - Ubuntu:22.04
> - Ros2:humble

## 在window中安装Unity

先安装UnityHub，然后再安装Unity3D

[https://unity.cn/releases](https://unity.cn/releases)

## 创建Docker容器，并安装相关软件

❇️创建Docker容器

```shell
docker run -it -p 6080:80 -p 10000:10000 -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=host.docker.internal:0.0 -e PULSE_SERVER=host.docker.internal --name=DockerUnityRos2Car333 docker.1ms.run/ubuntu:22.04  /bin/bash
```

❇️安装ros2

```shell
apt update

apt-get install sudo -y

sudo apt install software-properties-common -y

sudo add-apt-repository universe # 按后面Enter

sudo apt update && sudo apt install curl gnupg2 -y

sudo curl -sSL https://gitee.com/tyx6/rosdistro/raw/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt install curl gnupg2 -y

# 这一步先打开科学上网，不然会报错
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 这一步先关闭科学上网，不然会报错
sudo apt update && sudo apt upgrade -y

sudo apt install ros-humble-desktop -y
```

❇️添加终端启动sh

```shell
echo "source /opt/ros/humble/setup.bash" > ~/.bashrc
```


❇️安装必要的第三方库

```shell
sudo apt install python3-colcon-common-extensions -y   
```

## 运行测试

## Unity3D传感器数据获取测试

```shell
source install/setup.bash 
ros2 launch ros_tcp_endpoint endpoint.launch.py
```

```shell
source install/setup.bash
ros2 run topic_converter_python image_unconpressed
```

```shell
rviz2
```

# 参考：
> - [docker-ros2-unity-tcp-endpoint](https://github.com/frankjoshua/docker-ros2-unity-tcp-endpoint/tree/master)
> - [Robotics-Nav2-SLAM-Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example?tab=readme-ov-file)
> - [unity坐赛车游戏，简单三分钟了解一下](https://www.bilibili.com/video/BV1LU4y1o7re/?vd_source=3bf4271e80f39cfee030114782480463)
> - [How to Setup Unity and ROS2 in less than 5 minutes!](https://www.youtube.com/watch?v=1X6uzrvNwCk)
> - [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity)
> - [moveit2_yolobb_ws](https://github.com/laoxue888/moveit2_yolobb_ws)
> - [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
