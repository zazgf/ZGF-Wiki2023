# lidar_demo教程
## 系统环境
- ros melodic
## User Guide
### 编译
创建ｒｏｓ环境　～/catkin_ws/src\
把lidar_demo.zip　解压到～/catkin_ws/src目录下\
需要把～/catkin_ws/src/lidar_demo/src/lidar_demo.cpp里
``` 
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, callback);
中的"/points_raw" 更改为雷达数据源，即想要进行聚类识别的点云的ｔｏｐｉｃ
```
开始编译\
```cd ~/catkin_ws && catkin_make```
### 运行
```source devel/setup.bash```\
启动\
```roslaunch lidar_demo lidar_demo.launch```\
在ｌａｕｎｃｈ文件里有５个参数可以调节
```
 <!-- 雷达数据源-->
<arg name="topic_name" default="/points_raw"/> 
 <!-- rviz 显示时 Fixed Frame 处的名称-->
<arg name="frame_id" default="velodyne"/> 
 <!-- ０.01 是1分米-->
<arg name="Cluster_D" default="0.75"/>  
<!-- 识别物体包含的最少点数，少于此点数不划分-->
<arg name="Cluster_Min" default="20"/>  
<!-- 识别物体包含的最多点数，多于此点数不划分 (100000)-->
<arg name="Cluster_Max" default="1000"/> 
```
frame_id 必须和ｒｖｉｚ里的ｆixed frame 一致\
Cluster_D 是聚行半径，0.01是１分米\
Cluster_Min　识别物体包含的最少点数，少于此点数不划分\
Cluster_Max　识别物体包含的最多点数，多于此点数不划分\
Ｃluster参数根据我们识别需求可调

