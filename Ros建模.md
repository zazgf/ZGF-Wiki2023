# Ros建模
[robot_model](http://wiki.ros.org/robot_model/Tutorials)
## Create your own urdf file
### description: in this tutorial you start creating your own urdf robot description file.
- Create the tree structure\
  in this tutorial we will create the URDF description of the "robot" shown in the image below.
![image](images/link.png)\
The robot in the image is a tree structure.Let's start very simple,and create a description of that tree structure,without worrying about the dimensions etc.Fire up your favorite text editor,and create a file called my_robot.urdf:
```
   1 <robot name="test_robot">
   2   <link name="link1" />
   3   <link name="link2" />
   4   <link name="link3" />
   5   <link name="link4" />
   6 
   7   <joint name="joint1" type="continuous">
   8     <parent link="link1"/>
   9     <child link="link2"/>
  10   </joint>
  11 
  12   <joint name="joint2" type="continuous">
  13     <parent link="link1"/>
  14     <child link="link3"/>
  15   </joint>
  16 
  17   <joint name="joint3" type="continuous">
  18     <parent link="link3"/>
  19     <child link="link4"/>
  20   </joint>
  21 </robot>
```
So,just creating the structure is very simple.Now let's see if can get this urdf file parsed.There is a simple command line tool that will parse a urdf file for you, and tell you if the syntax is correct.\
you might need to install, urdfdom as an upstream,ROS independent package:\
```
sudo apt-get install liburdfdom-tools
```
Now run the check command:
```
rosmake urdfdom_model
check_urdf my_robot.urdf   
```
### Add the dimensions
So now that we have the basic tree structure.let's add the appropriate dimensions.As you notice in the robot image,the reference frame of each link (in green) is located at the bottom of the link.and is identical to the reference frame of the joint.So,to add dimensions to our tree,all we habe to specify is the offset from a link to the joint(s) of its children.To accomplish this,we will add the field<origin> to each of the joints.\
Let's look at the second joint.Joint2 is offset in the Y-direction from link1,a little offset in the negative X-direction from link1, and it is rotated 90 degrees around the Z-axis.So,we need to add the following<origin> element:
```
<origin xyz="-2 5 0" rpy="0 0 1.57"/>
```
If you repeat this for all the elements our URDF will look like this:
```
   1 <robot name="test_robot">
   2   <link name="link1" />
   3   <link name="link2" />
   4   <link name="link3" />
   5   <link name="link4" />
   6 
   7 
   8   <joint name="joint1" type="continuous">
   9     <parent link="link1"/>
  10     <child link="link2"/>
  11     <origin xyz="5 3 0" rpy="0 0 0" />
  12   </joint>
  13 
  14   <joint name="joint2" type="continuous">
  15     <parent link="link1"/>
  16     <child link="link3"/>
  17     <origin xyz="-2 5 0" rpy="0 0 1.57" />
  18   </joint>
  19 
  20   <joint name="joint3" type="continuous">
  21     <parent link="link3"/>
  22     <child link="link4"/>
  23     <origin xyz="5 0 0" rpy="0 0 -1.57" />
  24   </joint>
  25 </robot>
```
### Completing the Kinematics
What we didn't specify yet is around which axis the points rotate.Once we add that,we actually have a full kinematic model of this robot!All we need to do is add the <axis> element to each joint.The axis specifies the rotational axis in the local frame.\
So, If you look at joint2, you see it rotates around the positive Y-axis.So, simple add the following xml to the joint element:
```
<axis xyz="0 1 0" />
```
Similarly,joint1 is rotating around the following axis:
```
<axis xyz="-0.707 0.707 0"/>
```
Note that it is a good idea to normalize the axis.
If we add this to all the joints of the robot, our URDF looks like this:
```
   1 <robot name="test_robot">
   2   <link name="link1" />
   3   <link name="link2" />
   4   <link name="link3" />
   5   <link name="link4" />
   6 
   7   <joint name="joint1" type="continuous">
   8     <parent link="link1"/>
   9     <child link="link2"/>
  10     <origin xyz="5 3 0" rpy="0 0 0" />
  11     <axis xyz="-0.9 0.15 0" />
  12   </joint>
  13 
  14   <joint name="joint2" type="continuous">
  15     <parent link="link1"/>
  16     <child link="link3"/>
  17     <origin xyz="-2 5 0" rpy="0 0 1.57" />
  18     <axis xyz="-0.707 0.707 0" />
  19   </joint>
  20 
  21   <joint name="joint3" type="continuous">
  22     <parent link="link3"/>
  23     <child link="link4"/>
  24     <origin xyz="5 0 0" rpy="0 0 -1.57" />
  25     <axis xyz="0.707 -0.707 0" />
  26   </joint>
  27 </robot>
```
Update your file my_robot.urdf and run it through the parser.
```
check_urdf my_robot.urdf
```
That's it.you created your first URDF robot description!Now you can try to visualize the URDF using graphiz:
```
urdf_to_graphiz my_robot.urdf
```
and open the generated file with your favorite pdf viewer:
```
evince test_robot.pdf
```
![image](images/graphiz.png)
## Parse a urdf file
### Description: This tutorial teachs you how to use the urdf parser
### Reading a URDF file
This tutorial starts off where the previous one ended.You should still habe your my_robot.urdf file with a description of the robot shown before below.\
Let's first create a package with a dependency on the urdf parser in our snadbox:
```
cd ~/catkin_ws/src
catkin_create_pkg robot_description urdf roscpp rospy tf sensor_msgs std_msgs
cd robot_description
```
Now create a /urdf folder to store the urdf file we just created:
```
mkdir urdf
cd urdf
```
This follows the convention of always storing your robot's URDF file in a ROS package named MYROBOT_description and within a subfolder named /urdf.Other standard subfolders of your robot's description package include /meshes, /media and /cad,like so:\
```
/MYROBOT_description
  package.xml
  CMakeLists.txt
  /urdf
  /meshes
  /materials
  /cad
```
## Using the robot state publisher on your own robot
### Description: This tutorial explains how you can publish the state of your robot to tf, using the robot state publisher.
When you are working with a robot that has many relevant frames.it becomes quite a task to publish them all to tf.The robot state publisher is a tool that will do this job for you.\
![frame2](images/frames2.png)\
The robot state publisher helps you to broadcast the state of your robot to the tf transform library.The robot state publisher internally has a kinematic运动学 model of the robot; so given the joint positions of the robot,the robot state publisher can compute and broadcast the 3D pose of each link in the robot.
You can use the robot state publisher as a standalone ROS node or as a library:
### 1.Running as a ROS node
#### 1.1 robot_state_pubisher
The easiest way to run the robot state publisher is as a node.For normal users,this is the recommanded usage.You need two things to run the robot state publisher:
> - a urdf xml robot description loaded on the Parameter Server.
> - A source that publishes the joint positions as a sensor_msgs/JointState
##### 1.1.1 Subscribed topics
joint_states(Sensor_msgs/JointState)
> joint position information
##### 1.1.2 Parameters
robot_description(urdf map)\
tf_prefix(string)
> Set the tf prefix for namespace-aware publishing of transform
publish_frequency(double)
> Publish frequency of state publisher,default:50Hz
#### 1.2 Example launch file
```
<launch>
  <param name = "robot_description" textfile = "$(find mypackage)/urdf/robotmodel.xml"/>
  <node pkg = "robot_state_publisher" type="robot_state_publisher" name = "rob_st_pub">
  <remap from="robot_state_publisher" to="my_robot_description"/>
  <remap from="joint_states" to="different_joint_states"/>
  </node>
</launch>
```
### 2.Runing as a library
Advanced users can also run the robot state publisher as a library,from within their own c++ code.After you include the header:
```#include <robot_state_publisher/robot_state_publisher.h>```\
all you need is the constructor which takes in a KDL tree\
```RobotStatePublisher(const KDL::Tree& tree);```\
and now, everytime you want to publish the state of your robot, you call the publishTransforms functions:
```
//Publish moving joints
void publishTransfroms(const std::map<std::string, double>& joint_positions,
  const ros::Time& time);

//publish fixed joints
void publishFixedTransforms();
```

The first argument is a map with joint names and joint positions, and the second argument is the time at which the joint positions were recorded. It is okay if the map does not contain all the joint names. It is also okay if the map contains some joints names that are not part of the kinematic model. But note if you don't tell the joint state publisher about some of the joints in your kinematic model, then your tf tree will not be complete.

## Using urdf with robot_state_publisher
### Create the URDF File
### Publishing the State
> cd %TOP_DIR_YOUR_CATKIN_WS%/src

Then fire your favourite editor and paste the following code into the src/state_publisher.cpp file:
```
   1 #include <string>
   2 #include <ros/ros.h>
   3 #include <sensor_msgs/JointState.h>
   4 #include <tf/transform_broadcaster.h>
   5 
   6 int main(int argc, char** argv) {
   7     ros::init(argc, argv, "state_publisher");
   8     ros::NodeHandle n;
   9     ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  10     tf::TransformBroadcaster broadcaster;
  11     ros::Rate loop_rate(30);
  12 
  13     const double degree = M_PI/180;
  14 
  15     // robot state
  16     double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
  17 
  18     // message declarations
  19     geometry_msgs::TransformStamped odom_trans;
  20     sensor_msgs::JointState joint_state;
  21     odom_trans.header.frame_id = "odom";
  22     odom_trans.child_frame_id = "axis";
  23 
  24     while (ros::ok()) {
  25         //update joint_state
  26         joint_state.header.stamp = ros::Time::now();
  27         joint_state.name.resize(3);
  28         joint_state.position.resize(3);
  29         joint_state.name[0] ="swivel";
  30         joint_state.position[0] = swivel;
  31         joint_state.name[1] ="tilt";
  32         joint_state.position[1] = tilt;
  33         joint_state.name[2] ="periscope";
  34         joint_state.position[2] = height;
  35 
  36 
  37         // update transform
  38         // (moving in a circle with radius=2)
  39         odom_trans.header.stamp = ros::Time::now();
  40         odom_trans.transform.translation.x = cos(angle)*2;
  41         odom_trans.transform.translation.y = sin(angle)*2;
  42         odom_trans.transform.translation.z = .7;
  43         odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
  44 
  45         //send the joint state and transform
  46         joint_pub.publish(joint_state);
  47         broadcaster.sendTransform(odom_trans);
  48 
  49         // Create new robot state
  50         tilt += tinc;
  51         if (tilt<-.5 || tilt>0) tinc *= -1;
  52         height += hinc;
  53         if (height>.2 || height<0) hinc *= -1;
  54         swivel += degree;
  55         angle += degree/4;
  56 
  57         // This will adjust as needed per iteration
  58         loop_rate.sleep();
  59     }
  60 
  61 
  62     return 0;
  63 }
```
### Launch File
This launch file assumes you are using the package name "r2d2" and node name "state_publisher" and you have saved this urdf to the "r2d2" package.
```
   1 <launch>
   2         <param name="robot_description" command="cat $(find r2d2)/model.xml" />
   3         <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
   4         <node name="state_publisher" pkg="robot_description" type="state_publisher" />
   5 </launch>
```
### Viewing the Reult
First we have to edit the CMakeLists.txt in the package where we saved the above source code.Make sure to add the tf dependency in addition to the other dependencies:
> find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf)

Notice that roscpp is used to parse the code that we wrote and generate the state_publisher node. We also have to add the following to the end of the CMakelists.txt in order to generate the state_publisher node:
> include_directories(include ${catkin_INCLUDE_DIRS})\
add_executable(state_publisher src/state_publisher.cpp)\
target_link_libraries(state_publisher ${catkin_LIBRARIES})

Now we should go to the directory of the workspace and build it using:
> catkin_make

Now launch the package (assuming that our launch file is named display.launch):
> roslaunch r2d2 display.launch

Run rviz in a new terminal using:
> rosrun rviz rviz

Choose odom as your fixed frame (under Global Options). Then choose "Add Display" and add a Robot Model Display and a TF Display\
![image](images/Screenshot%20from%202020-11-03%2008-14-44.png)
## Building a Visual Robot Model with URDF from Scratch
### Description:learn how to build a visual model of a robot that you can view in rviz
Before continuing,make sure you have the joint_state_publisher package installed.
> sudo apt-get install ros-melodic-joint-state-publisher

### 1.One Shape
First, we're just going to explore one simple shape.Here's about as simple as a urdf as you can make.
```
<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```
To translate the XML into English, this is a robot with the name myfirst, that contains only one link (a.k.a. part), whose visual component is just a cylinder 0.6 meters long with a 0.2 meter radius. This may seem like a lot of enclosing tags for a simple “hello world” type example, but it will get more complicated, trust me.\

##　Gazebo 安装
`sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control`

## ros link 提示没有包含在gazebo
可能有两个原因，一个是它自身没有物理属性，另外一个是它的连接对象没有物理属性

## Denavit-Hartenberg parameters
In mechanical engineering, the Denavit-Hartenberg parameters(also called DH parameters) are the four parameters associated with a particular convention for attaching reference frames to the links of a spatial kinematic chain or robot manipulator.

Jacques Denavit and Richard Hartenberg introduced this convention in 1955 in order to standardlize the coordinate frames for spatial linkages.

**Denavit-Hartenberg convention**\
A commonly used convention for selecting frames of reference in robotics applications is Denavit and Hartenberg(D-H) convention. In this convention, coordinate frames are attached to the joints between two links such that one transformation is associated with the joint,[Z], and the second is associated with the link,[X]. The coordinate transformations along a serial robot consisting of n links from the kinematics equations of the robot.

![1](images/Screenshot%202021-02-03%2011:50:47.png)

where [T] is the transformation locating the end-link.

In order to determine the coordinate transformations [Z] and [X], the joints connecting the links are modeled as either hinged or sliding joints, each of which have a unique line S in space that forms the joint axis and define the relative movement of the two links.A typical serial robot is characterized by a sequence of six lines Si,i=1,......6, one for each joint in the robot.For each sequence of lines Si and Si+1, there is a common normal line Ai,j+1.The system of six joint axes Si