## ubuntu18 melodic
> sudo apt-get install\
python-pip python3-vcstool python3-pyqt4\
pyqt5-dev-tools\
libbluetooth-dev libspnav-dev\
pyqt4-dev-tools libcwiid-dev\
cmake gcc g++ qt4-qmake libqt4-dev\
libusb-dev libftdi-dev\
python3-defusedxml python3-vcstool\
ros-melodic-octomap-msgs       \
ros-melodic-joy                \
ros-melodic-geodesy            \
ros-melodic-octomap-ros        \
ros-melodic-control-toolbox    \
ros-melodic-pluginlib	      \
ros-melodic-trajectory-msgs    \
ros-melodic-control-msgs	      \
ros-melodic-std-srvs 	      \
ros-melodic-nodelet	      \
ros-melodic-urdf		      \
ros-melodic-rviz		      \
ros-melodic-kdl-conversions    \
ros-melodic-eigen-conversions  \
ros-melodic-tf2-sensor-msgs    \
ros-melodic-pcl-ros\
ros-melodic-navigation\
ros-melodic-sophus

> sudo pip install gym\
sudo apt-get install python-skimage\
sudo pip install h5py\
pip install tensorflow-gpu (if you have a gpu if not then just pip install tensorflow)\
sudo pip install keras

> cd ~\
git clone https://github.com/erlerobot/gym-gazebo\
cd gym-gazebo\
sudo pip install -e .

> cd gym-gazebo/gym_gazebo/envs/installation\
bash setup_melodic.bash


