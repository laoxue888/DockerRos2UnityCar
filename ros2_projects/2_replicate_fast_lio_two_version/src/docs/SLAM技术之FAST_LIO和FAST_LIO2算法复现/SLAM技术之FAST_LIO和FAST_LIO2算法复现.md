
---

# 前言

本文对FAST_LIO和FAST_LIO2算法复现

# 原理介绍


# 运行环境配置

❇️创建Docker容器

```shell
docker run -it -p 6080:80 -p 10000:10000 -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=host.docker.internal:0.0 -e PULSE_SERVER=host.docker.internal --name=DockerUnityRos2Car4 docker.1ms.run/ubuntu:22.04  /bin/bash
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


❇️安装第三方库

```shell
sudo apt-get install ros-humble-rqt-tf-tree
apt-get install gdb -y
apt-get install ros-${ROS_DISTRO}-pcl-ros
```


# 运行测试

## FAST_LIO算法运行测试

```shell
cd 2_replicate_fast_lio_two_version
source install/setup.bash
ros2 bag play /root/2_replicate_fast_lio_two_version/src/docs/place_data_here/rosbag2_2024_06_20-16_46_47
```

```shell
cd 2_replicate_fast_lio_two_version
source install/setup.sh
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

## FAST_LIO2算法运行测试

```shell
cd 2_replicate_fast_lio_two_version
source install/setup.bash
ros2 bag play /root/2_replicate_fast_lio_two_version/src/docs/place_data_here/rosbag2_2024_06_20-16_46_47
```

```shell
cd 2_replicate_fast_lio_two_version
source install/setup.bash 
ros2 launch fastlio2 lio_launch.py
```