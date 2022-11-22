本项目是Udacity自动驾驶课程的最后一个项目，通过ROS系统将感知、规划和执行模块连接在一起，并在Udacity提供的模拟器中完成自动驾驶任务。

准备

1. ROS系统：为了使用ROS系统，需要通过虚拟机中的LINUX系统安装，Udacity已经提高了完整的[虚拟机安装包](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip "点击下载")，并且配置好了所需要的环境（登录密码为 `udacity-nd`）。为了方便虚拟机和本地系统的通信，还可以下载[VirtualBox](https://www.virtualbox.org/wiki/Downloads)。
2. 模拟器：模拟器内置了一台自动驾驶汽车，可以通过ROS系统进行操作。其中ROS系统运行在虚拟机中，[模拟器](https://github.com/udacity/CarND-Capstone/releases)运行在Windows环境下，二者通过电脑端口进行通信。通信的方法参见 [Port Forwarding.pdf](Port+Forwarding.pdf)。
3. 另外，在项目源代码中包含了对交通信号灯进行训练所需的[训练集](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip)。
