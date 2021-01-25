### 引用其他包的ｍｓｇ
首先cmakelists.txt
```
find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_ros
  roscpp
  rospy
  std_msgs
  wjj
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime pcl_ros roscpp rospy std_msgs wjj
)

add_dependencies(teaching 
${catkin_EXPORTED_TARGETS}
wjj_gencpp)
```

再者package.xml
```
 <build_depend>wjj</build_depend>
 <exec_depend>wjj</exec_depend>
```

最后使用
```
#include <wjj/SaeJ1939.h>
```