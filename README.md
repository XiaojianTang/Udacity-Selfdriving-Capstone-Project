# 项目概述

本项目是Udacity自动驾驶课程的最后一个项目，通过ROS系统将感知、规划和执行模块连接在一起，并在Udacity提供的模拟器中完成自动驾驶任务。

# 准备

## 开发环境

1. ROS系统：为了使用ROS系统，需要通过虚拟机中的LINUX系统安装，Udacity已经提高了完整的[虚拟机安装包](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip "点击下载")，并且配置好了所需要的环境（登录密码为 `udacity-nd`）。为了方便虚拟机和本地系统的通信，还可以下载[VirtualBox](https://www.virtualbox.org/wiki/Downloads)。
2. 模拟器：模拟器内置了一台自动驾驶汽车，可以通过ROS系统进行操作。其中ROS系统运行在虚拟机中，[模拟器](https://github.com/udacity/CarND-Capstone/releases)运行在Windows环境下，二者通过电脑端口进行通信。通信的方法参见 [Port Forwarding.pdf](Port+Forwarding.pdf)。
3. 另外，在项目源代码中包含了对交通信号灯进行训练所需的[训练集](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip)。

进入虚拟机后需要先安装python所需要的库文件，相关库在 [requirements.txt](requirements.txt) 中，可以通过以下命令一次性获取：

```
cd CarND-Capstone
pip install -r requirements.txt
```

## ROS的基本操作

在完成部分代码后，可以在ROS中进行测试，编译和运行ROS代码的命令如下：

```
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

之后打开模拟器即可。

其他ROS相关的命令如下表（待补充）：

| 功能描述          | 命令行                                                 | 输出 |
| ----------------- | ------------------------------------------------------ | ---- |
| 开启主线程        | roscore                                                |      |
| 开启节点          | rosrun /node_name                                      |      |
| 查看 node 列表   | rosnode list                                           |      |
| 查看 topic 列表   | rostopic list                                          |      |
| 查看 msg 列表     | rosmsg list                                            |      |
| 查看 topic 信息   | rostopic info /topic_name                              |      |
| 查看 msg 信息     | rosmsg info /msg_type<br />rosed msg_name/msg_type.msg |      |
| 查看实时 msg 内容 | rostopic echo /topic_name                              |      |

## 项目计算图形

整个项目的架构如下图：

![1669173309753](image/README/1669173309753.png)

# Waypoint Updater Node - I

为了让车辆再模拟器中先运动起来，首先需要写下第一个版本的waypoint updater节点。此节点的计算图形如下：

![1669173382678](image/README/1669173382678.png)

python文件在 ` ros\src\waypoint_updater\waypoint_updater.py`，可[点击这里打开](ros\src\waypoint_updater\waypoint_updater.py)

这个节点最终目的是：**发布** **固定数量**的在车辆**前方**的 waypoint，每个waypoint需要包含考虑遇到**红绿灯**和**障碍物**时的情况正确的**目标速度**target velocity。

第一版可以暂时不用考虑交通灯和障碍物等的影响。完成后在模拟器中可以看到车辆前方有一串绿色路径点即可。后续完成了dbw(线控)的节点后，可以根据实际情况在更新本节点。

### 相关 Topic

需要 subscribe 的 topic 有：

* /base_waypoints  ：包含了道路上所有的基础 waypoints，只发布一次
* /current_pose  ：  当前车辆的姿态

需要 publish 的 topic 有：

* `/final_waypoints`

### Message 描述

在 `waypoint_updater.py` 中可以看出 `/base_waypoints` 和 `/final_waypoints` 传递的message 的类型是：

 `Lane`

关于 message 的描述可以在 `ros\src\styx_msgs\msg\Lane.msg` 里[查看](ros\src\styx_msgs\msg\Lane.msg)。包含了一个 `header` 和 数据类型为 `Waypoints` 的列表 ` waypoint[]`。更详细的信息也可以在命令行中用 ：

* `$ rostopic info /final_waypoints` 查看 topic 信息
* `$ rosmsg info styx_msgs/Lane` 查看 message 信息

`Lane `类型的 message 的信息如下：

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
styx_msgs/Waypoint[] waypoints
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  geometry_msgs/TwistStamped twist
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
```

可以看到除了 header 以外，还有一个 waypoint[] 的列表，别表中的每一个元素包含 pose 和 twist，pose 和 twist 下各自还有多个层级。如需获得 position 中的 x 坐标，可以用如下代码：

```
my_lane_msg[0]. pose.pose.position.x
```

如需获得x方向的速度信息，可以使用如下代码：

```
my_lane_msg[0].twist.twits.linear.x
```

### Topics 和 Msg 一览

| Topic            | Msg Type                  | 备注                                               |
| ---------------- | ------------------------- | -------------------------------------------------- |
| /base_waypoints  | styx_msgs/Lane            | 由 static.csv 文件获取                             |
| /current_pose    | geometry_msgs/PoseStamped | 车辆当前的位置和速度，由模拟器或定位模块获取       |
| /final_waypoints | styx_msgs/Lane            | 是/base_waypoints的一部分，在车前方离车最近的N个点 |

### 代码实现

#### `pose_cb()` 和 `waypoints_cb()`

首先需完成两个 callback 函数，分别用于暂存 pose 信息和 waypoint 信息：

* 其中在 `waypoints_cb()` 中，先需要将所有 waypoints 赋值给 `base_waypoints`，作为完整的 waypoints 集；
* 此外，需要将原本 waupoints 里的 x, y 坐标信息提取出来，赋值给 `waypoints_2d`；

  ```python
  self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
  ```
* 同时，为了方便后续能找到在当前位置前的 waypoints，需要使用 `scipy `库里的 `KDTree `数据结构。

  ```python
  self.waypoints_tree = KDTree(self.waypoints_2d)
  ```

> KDTree是一个k维的树形数据结构，主要用于在多维空间的数据搜索

#### `get_closest_waypoint_idx()`

在获取最近的 waypoint 时，我们希望得到的点是在车辆前方的，可以通过构建一些 hyperparameter 来实现，下图时通过**向量的点积**来判断最近点在是否在当前位置前方。

首先需要获取离当前位置 **[x,y]** 最近的 **1** 个点的**索引**，可是使用 `scipy.spatial` 库里 `KDTree `中的 `query()` 函数来获取，部分参考代码为：

```python
#get closest waypoint index using waypoints_tree
closest_idx = self.waypoints_tree.query([x,y],1)[1]
closest_coord = self.waypoints_2d[closest_idx]
```

当最近点在当前位置后方时，从“当前点指向最近点的向量 ` pos_vect - cl_vect`”与“前一个点到当前点的向量 `pos_vect - prev_vect`”方向相反，既**点积为负**，如下图所示。

![1669183000457](image/README/1669183000457.png)

部分参考代码如下：

```python
prev_coord = self.waypoints_2d[closest_idx-1]

#creat vectors to check is the waypoints in front of or behind current pose
cl_vect = np.array(closest_coord) # closest waypoint vector
prev_vect = np.array(prev_coord) # previous waypoint vector
pos_vect = np.array([x,y]) # current pose vector

# dot product > 0, means this waypoint is behind the current pose, then check next one
val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
if val>0:
    closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
```

#### `publish_waypoints()`

在__init__(self) 函数中定义 Publisher() 函数。后再 publish_waypoints() 函数中建立 lane 信息类型，并把 waypoints 传递给 lane，最后通过将传入发布函数完成发布，参数的传递路径如下：

**header, waypoints -> lane -> final_waypoints_pub()**

> 其中 `header `与 `base_waypoints `相同
>
> waypoints为在 `base_waypoints`中，从最近的点开始，数量为 `LOOKAHEAD_WP`的切片

参考代码如下：

```python
def __init__(self):
    ...
    self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
    ...

def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
```

### 初步测试

完成简单的waypoints的发布后，可以在模拟器中进行测试，这时候就可以看到 waypoints 已经被发布了，显示成绿色的小球：

![1669191866075](image/README/1669191866075.png)

# DBW Node

完成了获得 waypoint 后，接下来需要实现让车辆跟着waypoint运动，这需要完成通过 DBW 节点。DBW 是 Drive By Wire 的简写，意思是线控驾驶，是通过电控的方式，控制车辆的转向、油门、制动等。

## 相关 Topic

DBW 节点需要从 subscribe \twist_cmd 的 topic（由 waypoints_follower 节点 publish 自动完成），以获得车辆的目标位置、速度等信息，后将响应的操作指令 publish 以下 topic：

* /vehicle/throttle
* /vehicle/brake_cmd
* /vehicle/steering_cmd

> 此外，在测试阶段时，车辆通常配有安全员。当安全员介入车辆控制后，PID 等控制器应该停止累积误差，因此需要注意 DBW 是否正在工作

本节点相关的 topic 关系如下图：

![1669254140209](image/README/1669254140209.png)

## 相关源文件

DBW 节点涉及多个 .py 文件，主体部分在 dbw_node.py 中，其中可能需要应用到以下文件：

| 文件名                 | 说明                                                                               |
| ---------------------- | ---------------------------------------------------------------------------------- |
| dbw_node.py            | 配置节点的 subscriber 和 publisher，关系如上图                                     |
| > twist_controller.py | 包含 Controller 类，用于控制车辆                                                   |
| >> yaw_controller.py   | 可以用在 twist_controller 中的角度控制器，将线性和角度信息转化为**转向命令** |
| >> pid.py              | 可以用在 twist_controller 中的 PID 控制器                                          |
| >> lowpass.py          | 可以用在 twist_controller 中的 low pass 过滤器                                     |
| dbw_test.py            | 用于测试 DBW 节点                                                                  |

## 代码实现
