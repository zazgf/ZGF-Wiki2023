## 更改mac地址

* 使用ifconfig　查看网口
* 安装macmanager
  `sudo apt install macchanger`
* 随机生成一个mac地址
  `sudo macchanger -r enp60s0`
  `enp60s0`是网口，`-r`代表的是随机`random`的意思，`macchanger`会帮我们修改成一个随机产生的`MAC`号
* 修改为指定的mac地址
  `sudo macchanger -m AA:BB:CC:DD:EE:FF enp60s0`\

## ubuntu18.04 网口创建网络共享

- 终端输入nm-connection-editor打开网络连接
  ![1](images/20201224204459506.png)
- 创建以太网链接
  ![1](images/20201224211150665.png)
- 配置网络链接
  ![1](images/20201224210017800.png)
  ![1](images/20201224210814984.png)
- 将其他需要上网的设备通过网线链接到共享网络即可

## xvfb　ssh

`xvfb-run -s "-screen 0 1400x900x24" python <your_script.py>`

## x forward

ssh -X username@ip

## vscode 插件　Remote-ssh

conda install jupyter

## github下载提速

`git clone https://github.com/Amritpal-001/Reinforcement-learning-projects.git`
改为
`git clone http://hub.fastgit.org/Amritpal-001/Reinforcement-learning-projects.git`

## github 下载工具

[https://d.serctl.com/](https://d.serctl.com/)

## https://www.python.org/ftp 下载慢

`wget https://www.python.org/ftp/python/3.7.5/Python-3.7.5.tgz`
改为
`wget https://npm.taobao.org/mirrors/python/3.7.5/Python-3.7.5.tgz`

## pip 下载东西慢

`pip3.7.5 install -i http://mirrors.aliyun.com/pypi/simple/ psutil decorator numpy protobuf==3.11^C scipy sympy cffi grpcio grpcio-tools requests --user --trusted-host mirrors.aliyun.com`

## mkdocs

mkdocs build

mkdocs serve

push sites folders

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

## Tilix doesnt open in the folder from where it is split

Update `~.bashrc` (or `~.zshrc` if you are using zsh) to execute vte.sh directly, this involves adding the following line at the end of the file.

```
if [[ $TILIX_ID ]]; then
        source /etc/profile.d/vte.sh
fi
```

On Ubuntu (18.04), a symlink is probably missing. You can create it with:

```
ln -s /etc/profile.d/vte-2.91.sh /etc/profile.d/vte.sh
```

## sudo dist-upgrade 是毁灭性的，会升级cuda

## github init

```bash
git config --global user.email "you@example.com"
git config --global user.name "Your Name"
git add .
git commit -m "1"
git push
```

## 命令行打开文件管理器

`nautilus --browser ~/文档`

## 安装汉语输入法

先在设置里设置language里添加Chinese

```bash
sudo apt-get install ibus-libpinyin ibus-clutter
```

## Docker创建容器

### 构建ubuntu18.04映像

#### 安装依赖

```bash
sudo apt-get install debootstrap
sudo apt install docker.io
sudo chmod 666 /var/run/docker.sock
```

#### 创建ubuntu 18.04 镜像

```bash
sudo debootstrap bionic bionic > /dev/null
sudo tar -C bionic -c . | docker import - bionic/smart_eye
```

#### 测试

```bash
docker run bionic cat /etc/lsb-release
```

查看镜像

```bash
docker images
```

删除镜像

```bash
docker rmi [IMAGE ID]
```

查看容器运行情况

```bash
sudo docker ps -a
```

退出容器

```bash
sudo docker stop 容器id
```

删除容器

```bash
sudo docker rm 容器id
```

启动镜像

```bash
docker run -it bionic/smart_eye  /bin/bash
```

退出镜像

```bash
exit
```

文件传递

从本地至docker

```bash
docker cp FILE_PATH 容器ID:/root
```

从docker 至本地

```bash
docker cp 容器ID:/root/data.tar /home/user
```

#### 更改docker image存放路径

```bash
sudo service docker stop
sudo touch /etc/docker/daemon.json
```

daemon.json

```json
{
    "data-root":"/home/pmjd/docker"
}
```

```bash
sudo service docker start
```

## Docker更新apt source.list

```bash
sudo nano /etc/apt/sources.list
```

添加

```bash
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse

```

## Docker 启动容器

```bash
docker start 容器ID
docker attach 容器ID
```

导出容器快照

```bash
docker export c91c33f28594 > smart_eye_docker.tar
```

容器快照导入为镜像

```bash
cat smart_eye_docker.tar | docker import - test/smart_eye:v1
```

再保存此镜像

```bash
 docker save -o smart_eye_image_docker.tar test/smart_eye
```

再加载镜像

```bash
docker load --input samrt_eye_image_docker.tar
```

启动

```bash
docker run -it test/smart_eye:v1 /bin/bash
```

docker 启动bash

```bash
docker start c91c33f28594
docker exec -it c91c33f28594 /home/run.sh
docker exec mycontainer /bin/sh -c "cmd1;cmd2;...;cmdn"
```

docker bash file example

```bash
#!/bin/sh
docker run -it --net host --add-host in_release_docker:127.0.0.1 --add-host localhost:127.0.0.1 --hostname in_release_docker --rm promote/smart_eye:v1 /bin/bash -c "/home/run.sh"
```

启动快捷方式

1.desktop

```bash
[Desktop Entry]
Name=Smart_eye
GenericName=3D modeler
Keywords=3d;cg;modeling;animation;painting;sculpting;texturing;video editing;video tracking;rendering;render engine;cycles;game engine;python;
Exec=/bin/bash -c '/home/promote/run.sh'
#Icon=/home/pmjd/Disk/blender-2.90.1-linux64/blender.svg
Terminal=true
Type=Application
Categories=Graphics;3DGraphics;
MimeType=application/x-blender;
```

界面显示

```bash
xhost +
-e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix
```

## 安装NVIDIA Container Toolkit

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```

```bash
curl -s -L https://nvidia.github.io/nvidia-container-runtime/experimental/$distribution/nvidia-container-runtime.list | sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
```

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
```

这个时候`/etc/docker/daemon.json` 内容修改为

```bash
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "data-root":"/home/pmjd/docker"
}
```

重启docker

```bash
sudo systemctl restart docker
```

下载运行：

```bash
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
sudo docker run --rm --gpus all nvidia/cuda:10.1-base nvidia-smi
```

下载运行tensorflow:

```bash
docker run --gpus all --runtime=nvidia -it tensorflow/tensorflow:2.3.0-gpu bash
```

docker配置网络

```bash
--net=host 
```

docker 支持麦克,汉语

```bash
docker run -it --volume=/run/user/1000/pulse:/run/user/1000/pulse --user promote --gpus all --runtime=nvidia --name=xiaomeng -e LANG=C.UTF-8 --device /dev/snd promote/xiaomeng:v1.6 /bin/bash
```

## Nano 删除行

```bash
ctrl+k
```

Nano显示行号

```bash
alt+shift+3
```

### 修改默认python版本

删除/usr/bin 下的Python链接

```bash
sudo rm /usr/bin/python
```

用下面命令建立新的链接

```bash
sudo ln -s /usr/bin/python3.6 /usr/bin/python
```

用下面的命令还原2版本

```bash
sudo ln -s /usr/bin/python2.7 /usr/bin/python
```

更科学的做法是：

```bash
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1

sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2

sudo update-alternatives --config python 选择要使用的版本，回车，搞定
```

### 设置默认pip版本

```bash
sudo update-alternatives --install /usr/bin/pip pip /usr/bin/pip2 1

sudo update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 2

sudo update-alternatives --config pip
```

### pip升级

```bash
sudo pip install --upgrade pip
```

### 桌面快捷方式

```bash
[Desktop Entry]
Name=smart_view
GenericName=3D modeler
Keywords=python;
Exec=/bin/bash -c 'source /opt/ros/melodic/setup.bash;rosrun image_view image_view image:=/smart_eye_view'
#Icon=/home/pmjd/Disk/blender-2.90.1-linux64/blender.svg
Terminal=false
Type=Application
Categories=Graphics;3DGraphics;
MimeType=application/x-blender;
Name[en_US]=smart_view
```

### md公式里添加空格

`\quad`或者`\+空格` 或者`&nbsp`

### md添加多行公式

`![](https://latex.codecogs.com/svg.latex?\Large&space;\theta_{k+1}=arg\quad\underset{\theta}{max})`

![](https://latex.codecogs.com/svg.latex?\Large&space;\theta_{k+1}=arg\quad\underset{\theta}{max})

### md添加行内公式

`$\theta$` $\theta$
