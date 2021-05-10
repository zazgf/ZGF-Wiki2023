## ubuntu18 melodic

> sudo apt-get install
> python-pip python3-vcstool python3-pyqt4
> pyqt5-dev-tools
> libbluetooth-dev libspnav-dev
> pyqt4-dev-tools libcwiid-dev
> cmake gcc g++ qt4-qmake libqt4-dev
> libusb-dev libftdi-dev
> python3-defusedxml python3-vcstool
> ros-melodic-octomap-msgs
> ros-melodic-joy
> ros-melodic-geodesy
> ros-melodic-octomap-ros
> ros-melodic-control-toolbox
> ros-melodic-pluginlib
> ros-melodic-trajectory-msgs
> ros-melodic-control-msgs
> ros-melodic-std-srvs
> ros-melodic-nodelet
> ros-melodic-urdf
> ros-melodic-rviz
> ros-melodic-kdl-conversions
> ros-melodic-eigen-conversions
> ros-melodic-tf2-sensor-msgs
> ros-melodic-pcl-ros
> ros-melodic-navigation
> ros-melodic-sophus

> sudo pip install gym
> sudo apt-get install python-skimage
> sudo pip install h5py
> pip install tensorflow-gpu (if you have a gpu if not then just pip install tensorflow)
> sudo pip install keras

> cd ~
> git clone https://github.com/erlerobot/gym-gazebo
> cd gym-gazebo
> sudo pip install -e .

> cd gym-gazebo/gym_gazebo/envs/installation
> bash setup_melodic.bash

### Tensorflow 算法功能

1. 记录仪设置
2. 随机种子设置
3. 环境实例化
4. 为计算图制作占位符
5. actor_critic 通过作为参数传递给算法函数来构建参与者评论计算图
6. 实例化体验缓冲区
7. 建立特定于算法的损失函数和诊断的计算图
8. 进行培训
9. 进行tf会话并初始化参数
10. 通过记录器设置模型保存
11. 定义运行算法主循环所需的功能（例如，取决于算法，核心更新功能，获取操作功能和测试代理功能）
12. 运行算法的主循环：
    1. 在环境中运行代理
    2. 根据算法的主要方程式定时更新代理参数
    3. 记录关键性能指标并保存代理

核心文件

核心文件不像算法文件那样紧密地附着在模版上，但具有一些近似的结构：

1. 仅tensorflow:与制作和管理占位符有关的功能
2. 用于建立actor_critic 特定算法的方法相关的计算图部分的功能
3. 任何其他有用的功能
4. 于算法兼容的MLP actor_critic 的实现，其中策略和值函数都由简单的MLP表示。

体验深度RL的最佳方法之一是运行算法，并查看它们在不同任务上的执行情况。Spinning Up随附spinup/run.py,这是一个方便的工具，可以让你从命令行轻松的启动各种好算法（可以选择超参数）

### 近端策略优化

PPO受到与TRPO相同问题的激励：我们如何才能使用当前拥有的数据在策略上采取最大可能的改进步骤，而又不会走的太远而意外导致性能下降。TRPO尝试使用复杂的二阶方法解决此问题。PPO是一阶方法系列，它使用其他一些技巧来使新策略接近于旧策略。PPO方法明显更易于实现，并且从经验上看，其性能至少与TRPO相同。PPO有两种主要变体：PPO-Penalty， PPO-Clip。

PPO-Penalty 大约解决了像TRPO这样受KL约束的更新。但是会惩罚目标函数中的KL散度，而不使其成为硬约束，并会在训练过程中自动调整惩罚系数，以使其适当缩放。

PPO-Clip 在目标中没有KL散度项， 也没有任何约束。取而代之的是依靠对目标函数的专门裁剪来消除新政策远离旧政策的动机。

#### 要闻速览

* PPO是一种基于策略的算法
* PPO可以应用于具有离散的或连续的动作空间的环境
* PPO的Spinningup实现支持与MPI并行化

PPO更新策略公式

$$
\theta_{k+1}=arg\underset{\theta}{max}\underset{s,\alpha\sim\pi\theta_k}{E}[L(s,\alpha,\theta_k,\theta)]

$$

通常采取多个步骤（通常是小批量）SGD来最大化目标

$$
L(s,\alpha,\theta_k,\theta)=min(\frac{\pi_\theta(\alpha|s)}{\pi_{\theta_k}(\alpha|s)}A^{\pi\theta_k}(s,\alpha),clip(\frac{\pi_\theta(\alpha|s)}{\pi_{\theta_k}(\alpha|s)},1-\epsilon,1+\epsilon)A^{\pi\theta_k}(s,\alpha))

$$

$\epsilon$是一个超参数粗略地说出新政策被允许与旧政策相距多远

这是一个很复杂的方程式，第一眼很难说明白它在做什么，或者它如何帮助新策略靠近旧策略。事实证明有一个相当简化的版本，较容易理解，也是我们在代码中实现的版本

$$
L(s,\alpha,\theta_k,\theta) = min\left(\frac{\pi_\theta(\alpha|s)}{\pi_{\theta_k}(\alpha|s)}A^{\pi\theta_k}(s,\alpha), g\left(\epsilon,A^{\pi\theta_k}(s,\alpha)\right)\right)

$$

$$
g(\epsilon,A)=\left\{\begin{matrix}(1+\epsilon)A\quad A\geq0&\\(1-\epsilon)A\quad A<0\end{matrix}\right.

$$

为了弄清楚从中得到的直觉，让我看一下单个的 状态—动作 对，并考虑案例

**优势是积极**的，假设该状态—动作 对的优势为正，在这种情况下，它对目标的贡献减小为：

$$
L(s,\alpha,\theta,\theta_k)=min\left(\frac{\pi_\theta(\alpha|s)}{\pi_{\theta_k}(\alpha|s)},(1+\epsilon)\right)A^{\pi\theta_k}(s,\alpha)

$$

因为优势是积极的，所以如果采取行动的可能性更大（也就是$\pi_{\theta}(\alpha|s)$增加），目标就会增加。但是term中的min限制了可以增加多少目标。一旦$\pi_\theta(\alpha|s)>(1+\epsilon)\pi_{\theta_k}(\alpha|s)$,这时min踢进，term到达$(1+\epsilon)A^{\pi\theta_k}(s,\alpha)$,因此用过远离旧政策，新政策不会获益。

**优势是负面**的，假设该状态—动作 对的优势是负的，在这种情况下，它对目标的贡献减小为：

$$
L(s,\alpha,\theta,\theta_k)=max\left(\frac{\pi_\theta(\alpha|s)}{\pi_{\theta_k}(\alpha|s)},(1-\epsilon)\right)A^{\pi\theta_k}(s,\alpha)

$$

因为优势是负的，所以如果 采取行动的可能性更小（也就是$\pi_\theta(\alpha|s)$减小），目标就会增加。但是term中的max限制了可以增加多少目标。一旦$\pi_\theta(\alpha|s)<(1-\epsilon)\pi_{\theta_k}(\alpha|s)$,这时max踢进，term到达$(1-\epsilon)A^{\pi\theta_k}(s,\alpha)$,因此，再次，远离旧政策，不会使新政策受益。

到目前为止，通过去除使政策发生重大变化的激励，裁剪充当着正则化器。而超参数$\epsilon$则对应于新政策与旧政策可以有多远，同时仍使目标受益。

## 探索与发现

PPO按基于策略方式训练随机策略，这意味着它会根据其随机策略的最新版本通过采样操作来进行探索。动作选择的随机性取决于初始条件和训练步骤，在训练过程中，代理逐步减少随机性，因为更新规则鼓励它利用已经发现的奖励。这可能导致策略陷入局部最优状态。

### 伪代码

---

PPO-CLIP

---

输入：初始策略参数$\theta_0$,初始值功能参数$\phi_0$

for k = 0, 1, 2, ... do

   在环境中通过执行策略$\pi_k=\pi(\theta_k)$，收集一组轨迹$D_k=\{\tau_i\}$

   算奖励$\hat{R}_i$

   基于当前的value function $V_{\phi k}$,计算advantage estimates $\hat{A}_t$

   通过最大化PPO-Clip目标，更新策略

$$
\theta_{k+1}=arg\ \underset{\theta}{max}\frac{1}{|D_k|T}\sum_{\tau\in D_k}\sum^T_{t=0}min\left(\frac{\pi_\theta(\alpha_t|s_t)}{\pi_{\theta_k}(\alpha_t|s_t)}A^{\pi\theta_k}(s_t,\alpha_t),g(\epsilon,A^{\pi\theta_k}(s_t,\alpha_t))\right)

$$

   通常通过与Adam一起的随机梯度上升

   通过均方误差回归拟合value function

$$
\phi_{k+1}=arg\ \underset{\phi}{min}\frac{1}{|D_k|T}\sum_{\tau\in D_k}\sum^T_{t=0}(V_\phi(s_t)-\hat{R}_t)^2

$$

   特别是通过一些梯度下降算法

end for

---
