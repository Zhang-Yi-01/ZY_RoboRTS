# 关于定位接口
定位包中，运行定位结点

```
roslaunch robot_localization localizatio_node.launch
```
#### input: 仿真/真实雷达（以及定位imu）的数据
#### output: 对应的定位信息，话题为fused_localization,发送的数据消息格式为来自ros包的nav_msgs::Odometry（仿真）,包含的内容具体如下
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
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
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```
  其中geometry_msgs/Vector3 angular角速度始不给，置为零。
当雷达与定位结点运行时，可用ros话题通信接收，可参考``robot_localization/src/subscribe/imu_subscriber.cpp``
