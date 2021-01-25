#### 更改mac地址
* 使用ifconfig　查看网口
* 安装macmanager\
  `sudo apt install macchanger`
* 随机生成一个mac地址\
  `sudo macchanger -r enp60s0`\
  `enp60s0`是网口，`-r`代表的是随机`random`的意思，`macchanger`会帮我们修改成一个随机产生的`MAC`号
* 修改为指定的mac地址\
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
改为\
`git clone http://hub.fastgit.org/Amritpal-001/Reinforcement-learning-projects.git`