## Atlas200验收报告
-- -- 
**在线支持**
- [晟腾学院](https://www.huaweicloud.com/ascend/home.html)
- [Atlas 200 DK](https://support.huaweicloud.com/twp-atlas200dkappc32/atlasug_02_0003.html)


**性能特点**
- 可提供16TOPS (INT8) 的峰值计算能力。
- 支持两路Camera输入，两路ISP图像处理，支持HDR10高动态范围技术标准
- 支持1000M以太网对外提供高速网络连接，匹配强劲计算能力
- 通用的40-pin扩展接口，方便产品原型设计
- 支持5v~28v宽范围直流电源输入

**需要配件**
- micro sd卡 32G
- typeC usb 线
- 网线
- 读卡器
- Ubuntu服务器

**制作Usb系统启动盘**
- 将SD卡放入读卡器，将读卡器与Ubuntu服务器的USB连接，然后通过制卡脚本进行SD卡的制作
- 软件包准备
  - 制卡入口脚本  [make_sd_card.py](https://www.huaweicloud.com/ascend/resource/Software)
  - 制作SD卡操作系统脚本 [make_ubuntu_sd.sh](https://www.huaweicloud.com/ascend/resource/Software)
    - 请选择配套的CANN版本：20.1。
    - “产品系列”请选择“Atlas 200 DK”。
    - “产品型号”请选择“Atlas 200 DK 开发者套件”。
  - [Ubuntu操作系统镜像包](http://releases.ubuntu.com/releases/)
    - ubuntu-18.04.xx-server-arm64.iso
  - [开发者板驱动包](https://www.huaweicloud.com/ascend/resource/Software)
    - A200dk-npu-driver-{software version}-ubuntu18.04-aarch64-minirc.tar.gz
    - 请选择配套的CANN版本：20.1。
    - “产品系列”请选择“Atlas 200 DK”。
    - “产品型号”请选择“Atlas 200 DK 开发者套件”。
  - [开发者板运行包](https://www.huaweicloud.com/ascend/cann-download)
    - Ascend-cann-minirc_{software version}_ubuntu18.04-aarch64.zip
    - 请从“CANN软件包”中选择对应版本的软件包下载。
  
**操作步骤**
1. 请将SD卡放入读卡器，并将读卡器与Ubuntu服务器的USB接口连接。
2. `sudo apt-get install qemu-user-static binfmt-support python3-yaml gcc-aarch64-linux-gnu g++-aarch64-linux-gnu`
3. 把准备好的文件包放在/home/username/mksd目录下，`cd /home/username/mksd`
4. 执行`fdisk -l`查看sd卡所在usb设备名称为“/dev/sda”，可通过插拔SD卡的方式确定设备名称。
5. 运行SD制卡脚本“make_sd_card.py”。
   1. `python3 make_sd_card.py local /dev/sda`
   2. “local”表示使用本地方式制作SD卡。
   3. “/dev/sda”为SD卡所在的USB设备名称。

**遇到问题**：制作sd卡不成功，经过反复协助华为方技术人员，发现华为官方提供的脚本文件`make_ubuntu_sd.sh`有bug。使用体验差。华为方解决问题周期长，这个问题用了两天才解决。后期应用过程中华为atlas平台也会存在一些bug，开发速度受限于华为方解决atlas产品bug的速度。遇到Atlas平台问题，只能在华为云论坛上进行提问，解决问题周期长。

6. 获得修正后的`make_ubuntu_sd.sh`脚本文件安装成功。

Atlas200 开机启动成功，进入系统。\
**进行服务器ip设置**：\
ip:192.168.0.3\
netmask:255.255.0.0\
gateway:192.168.0.1\
**ssh登入Atlas200**:\
`ssh HwHiAiUser@192.168.0.2`\
密码:\
`Mind@123`\

