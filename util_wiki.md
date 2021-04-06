# rqt_image_view 提示 ImageView.callback_image() could not convert image from '8UC3' to 'rgb8' ([8UC3] is not a color format. but [rgb8] is. The conversion does not make sense)

解决办法:\
- 首先在ｏｐｅｎｃｖ处把８ＵＣ３格式的ｍａｔ转化为ｒｇｂ格式

```cv::cvtColor(compoundFrame, compoundFrame, cv::COLOR_RGB2BGR);```
- 然后cv_bridge编码格式改为sensor_msgs::image_encodings::RGB8

```img_bridge = cv_bridge::CvImage(headers, sensor_msgs::image_encodings::RGB8, compoundFrame);```

# Ros自定义ｍｓｇ编译时报错：缺少头文件
只需在CMakeLists.txt里添加\
```add_dependencies(GetLaneExtDemo smart_eye_gencpp)```\
smart_eye 可换成任意pkg 名称\
GetLaneExtDemo　为节点名称

# ModuleNotFoundError: No module named ‘rospkg’
```
pip install rospkg                       //更新方式1
sudo apt-get install python-rospkg           //更新方式2
//网上说有的方式1能解决，有的方式2可以解决，用pip更新的前提是安装了pip
```
# ImportError: No module named genmsg
命令：把sudo make 改为　make,就可以找到库，目前原因不明

# ui_mainwindow.h: No such file or directory
`set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)`

# git的命令行应用
> 1. git status
> 2. git add .
> 3. git commit -m "q"
> 4. git push

## 虽然有不同的ｒｏｓ工作空间
但是最好不要有相同的节点名字。因为容易引起混乱，运行混乱等。。。
