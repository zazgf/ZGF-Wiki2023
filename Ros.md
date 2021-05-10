## 引用其他包的ｍｓｇ

首先cmakelists.txt

```css
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

## sudo rosdep init &rosdep update失败

该解决方案是针对由于以下两个无法正常访问，但可以ping通，于是修改hosts文件，加入以下两个网址的IP地址实现访问。

'sudo gedit /etc/hosts'

添加

```bash
199.232.28.133  raw.githubusercontent.com
151.101.228.133  raw.github.com

```

修改完成后，在终端执行

```bash
sudo rosdep init
redep update
```

## vscode python ros debug

首先

```bash
catkin_make -DCMAKE_BUILD_TYPE=DEBUG
```

其次点击debug按钮，选择生成新的launch文件。

ｄebug时遇到路径问题,例如rosmsg路径，最好在文件属性里复制路径，不然容易出错

```python
import sys
sys.path.append("/home/pmjd/Downloads/catkin_ws/devel/lib/python2.7/dist-packages")
```

## Error: package 'teleop_twist_keyboard' not found

You need to download the teleop_twist_keyboard from the github to your ~/catkin_ws/src folder. Steps:

1) `cd ~/catkin_ws/src`
2) `git clone https://github.com/ros-teleop/teleop_twist_keyboard`

## Spawn service failed. Exiting.

```bash
export ROS_MASTER_URI=http://promote-OMEN-by-HP-Laptop-17-cb1xxx:11311/
```

## createQuaternionFromRPY

static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
geometry_msgs::Quaternion q;
double t0 = cos(yaw * 0.5);
double t1 = sin(yaw * 0.5);
double t2 = cos(roll * 0.5);
double t3 = sin(roll * 0.5);
double t4 = cos(pitch * 0.5);
double t5 = sin(pitch * 0.5);
q.w = t0 * t2 * t4 + t1 * t3 * t5;
q.x = t0 * t3 * t4 - t1 * t2 * t5;
q.y = t0 * t2 * t5 + t1 * t3 * t4;
q.z = t1 * t2 * t4 - t0 * t3 * t5;
return q;
}

## launch 启动 rviz

```bash
<launch>
  <node type="rviz" name="rviz" pkg="rviz" args="-d $(find package_name)/rviz/config_file.rviz" />
</launch>
```

## rosdep失败

首先`sudo rosdep init`

这一步会在`/etc/ros/rosdep/sources.list.d/`目录下新建`20-default.list`

```bash
# os-specific listings first
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

# generic
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead

```

我们需要依次下载`osx-homebrew.yaml`等这几个`yaml`文件,[下载工具](https://d.serctl.com/), 存放在/home/promote/Downloads目录下

然后更改`20-default.list`为

```bash
# os-specific listings first
yaml file:///home/promote/Downloads/2021-04-01-14-01-21-master-osx-homebrew.yaml osx

# generic
yaml file:///home/promote/Downloads/2021-04-01-14-02-25-master-base.yaml
yaml file:///home/promote/Downloads/2021-04-01-14-05-35-master-python.yaml
yaml file:///home/promote/Downloads/2021-04-01-14-06-50-master-ruby.yaml
gbpdistro file:///home/promote/Downloads/2021-04-01-14-07-41-master-fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead

```

还需要使用`mate-search-tool` 在`/usr/lib/`目录下找到包含`DEFAULT_INDEX_URL` 的`py`文件。`/usr/lib/python2.7/dist-packages/rosdistro/__init__.py`

找到代码行

```python
DEFAULT_INDEX_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml'
```

同样我们下载`index-v4.yaml`文件至`/home/promote/Downloads`，把此行代码改为

```python
# DEFAULT_INDEX_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml'
DEFAULT_INDEX_URL = 'file:/home/promote/Downloads/2021-04-01-14-51-42-rosdistro-index-v4.yaml'

```

这时再运行`rosdep update`

会提示`No such file or directory: '/home/promote/Downloads/dashing/distribution.yaml'`

我们需要下载`https://raw.githubusercontent.com/ros/rosdistro/master/dashing/distribution.yaml`

在`/home/promote/Downloads`目录下新建文件夹`dashing`，并把下载的`yaml`文件放入`dashing`文件夹下重命名为`distribution.yaml`

重复上一步，依次下载完`dashing, kinetic, melodic, rolling, noetic, foxy`等

再运行`rosdep update`就成功了

## ros node 打包

### 安装依赖

Install [bloom](http://ros-infrastructure.github.io/bloom/):

```shell
sudo apt-get install python-bloom
```

or (recommended)

```shell
sudo pip install -U bloom
```

Install fakeroot:

```shell
sudo apt-get install fakeroot
```

### 准备

To make a debian folder structure from the ROS package you must cd into the package to be in the same folder where `package.xml` file is.

### 生成debian包

```bash
bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
```

### ros总是链接anaconda下的库文件

```bash
export LD_LIBRARY_PATH=""
```
