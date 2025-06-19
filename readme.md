
---

## 前言

本项目介绍一种基于**Unity3D**引擎的**SLAM**多传感器仿真平台，可模拟 **IMU、RGB-D相机、激光雷达**传感器数据输出，为SLAM算法提供**低成本、高可控性、可复现**的仿真测试环境。

![alt text](images/rgbd_lidar.gif)

video:
> - []()
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

![alt text](images/test.gif)

## 搭建开发环境（Setup Development Environment）

> - Unity:2020
> - Ubuntu:24.04
> - Ros2:jazzy

## 在window中安装Unity（Install Unity in window）

先安装Unityhuyb，然后再安装Unity

[https://unity.cn/releases](https://unity.cn/releases)

## 创建Docker容器，并安装相关软件（Create Docker containers and install related software）

❇️创建Docker容器

```shell
docker run -it -p 6080:80 -p 10000:10000 -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=host.docker.internal:0.0 --gpus=all --name=Ros2UnityCar docker.1ms.run/ubuntu:24.04  /bin/bash
```

😂【可选】介绍另外一种方法：借助Docker的网页桌面功能，可以不用安装vncserver。[docker-webtop](https://github.com/linuxserver/docker-webtop)

```shell
# 可选
docker run -d --name Ros2UnityCar --security-opt seccomp=unconfined --gpus=all -e PUID=1000 -e PGID=1000 -e TZ="Asia/Shanghai" -p 3000:3000 -p 3001:3001 -p 10000:10000 lscr.io/linuxserver/webtop:ubuntu-xfce # ubuntu:24.04 科学上网下载速度更快

# 查看Ubuntu版本的命令
lsb_release -a
```

❇️安装相关软件

```shell
# 按照鱼香ros一键安装ros2
apt-get update
apt install wget -y
wget http://fishros.com/install -O fishros && bash fishros

# 打开新的终端，安装gz
sudo apt-get update
sudo apt-get install curl lsb-release gnupg -y
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update -y
sudo apt-get install gz-harmonic -y

# 安装远程显示服务程序
apt-get install x11-xserver-utils
apt install libxcb* -y
apt-get install x11-apps -y

# 安装moveit
apt install ros-${ROS_DISTRO}-moveit* -y

# 安装ros2的控制功能包
sudo apt install ros-${ROS_DISTRO}-controller-manager -y
sudo apt install ros-${ROS_DISTRO}-joint-trajectory-controller -y
sudo apt install ros-${ROS_DISTRO}-joint-state-broadcaster -y
sudo apt install ros-${ROS_DISTRO}-diff-drive-controller -y

# 安装其他功能包
# apt install ros-${ROS_DISTRO}-ros-gz -y
apt-get install ros-${ROS_DISTRO}-joint-state-publisher-gui -y
apt install ros-${ROS_DISTRO}-moveit-ros-planning-interface -y
# apt install ros-jazzy-gz-ros2-control 这个很重要 https://github.com/ros-controls/gz_ros2_control
apt install ros-${ROS_DISTRO}-gz-ros2-control -y

# 用于调试，可不安装
apt-get install gdb -y

# 安装python第三方库
apt install python3-pip -y
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
pip install pyside6 xacro ultralytics NodeGraphQt --break-system-packages
pip install -U colcon-common-extensions vcstool --break-system-packages

pip install pygame --break-system-packages
```

## 运行测试（Run test）

❇️打开Unity项目

![alt text](images/image.png)

![alt text](images/image-1.png)

❇️打开ROS2项目

![alt text](images/image-2.png)


![alt text](images/test.gif)
