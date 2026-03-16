# IMU 配置记录

## 0. 当前目标与边界

当前 IMU 接入按两个阶段做，不要混在一起：

1. 电脑端厂家层 bring-up
2. 小车上位机（工控机）实机校验

本文当前只覆盖这两件事：

- 在自己的电脑上把厂家 IMU 驱动跑起来，看到 `/imu/data`
- 在上位机上确认 `轴向 / 符号 / frame_id / 静止输出`

本文暂时**不把 IMU 直接接进 `scout_local_planner` 作为完成标准**。  
原因是你当前项目里的 IMU 接口虽然已经预留，但 `LocalPlannerROS` 现在是**直接读取**：

- `linear_acceleration.y`
- `angular_velocity.z`
- `angular_velocity.z` 差分得到的 `alpha_z`

当前代码**没有做 TF 旋转，也没有做重力补偿**。因此：

- 如果 IMU 安装方向和 `base_link` 不一致，当前代码不会自动帮你转正
- 如果 `linear_acceleration` 含重力分量，当前代码也不会自动剔除

所以，**在上位机端把轴向和静止输出核清之前，不要直接打开 `slosh_use_imu_*`。**

---

## 1. 电脑端阶段的完成标准

在你自己的电脑上，只要做到下面几条，就算厂家层 bring-up 成功：

```bash
ll /dev/imu_usb
rostopic list | grep imu
rostopic echo -n1 /imu/data
rostopic hz /imu/data
```

如果当前机器还没把 `udev` 规则修好，也可以接受下面这条临时路线：

```bash
ls /dev/ttyUSB*
rostopic list | grep imu
rostopic echo -n1 /imu/data
rostopic hz /imu/data
```

建议补看：

```bash
rostopic echo -n1 /wit/mag
```

这一阶段的目标只有一个：

- IMU 驱动正常发布 `/imu/data`

这一阶段**不要求**：

- 接入 `scout_local_planner`
- 验证 `base_link` 轴向
- 打开 `slosh_use_imu_lateral_accel`
- 打开 `slosh_use_imu_yaw_rate`
- 打开 `slosh_use_imu_alpha_z`

---

## 2. 电脑端 bring-up 流程（ROS1）

厂家 PDF 给了完整的 ROS1 路线。当前项目也是 ROS1，因此先按 ROS1 做。

### 2.1 先确认 ROS1 环境

```bash
echo $ROS_DISTRO
```

如果这里不是 ROS1 发行版，例如 `noetic` / `melodic` / `kinetic`，先不要往下继续。

### 2.2 安装依赖

如果你电脑是 Ubuntu 20.04 + ROS Noetic：

```bash
sudo apt-get update
sudo apt-get install ros-noetic-imu-tools ros-noetic-rviz-imu-plugin
pip3 install pyserial
sudo apt-get install ros-$ROS_DISTRO-serial
```

如果你不是 Noetic，把 `noetic` 改成你的实际发行版。

如果 `pip3 install pyserial` 提示：

```text
Requirement already satisfied
```

说明系统里已经有 `pyserial`，直接继续后面的步骤即可。

### 2.3 解压厂家 ROS1 资料包

当前你的厂家 ROS1 资料包路径是：

```text
/home/a/scout_ws/src/ros1_imu.zip
```

建议解压到用户主目录：

```bash
cd ~
unzip /home/a/scout_ws/src/ros1_imu.zip
```

解压后确认目录存在：

```bash
ls ~
```

正常应看到：

```text
wit_ros_imu
```

### 2.4 先做 USB 自动绑定

```bash
cd ~/wit_ros_imu
sudo bash bind_usb.sh
```

然后：

1. 拔掉 IMU
2. 重新插上 IMU
3. 检查绑定结果

```bash
ll /dev/imu_usb
```

如果能看到 `/dev/imu_usb`，后面优先用它，不要长期依赖 `/dev/ttyUSB0`。

### 2.5 如果自动绑定失败，检查 `lsusb` 和 rules

按厂家文档做两次 `lsusb`：

```bash
lsusb
# 先记一次，不插 IMU

# 插上 IMU 后再执行一次
lsusb
```

对比前后差异，确认厂商 ID / 产品 ID。

然后编辑规则文件：

```bash
cd ~/wit_ros_imu
vim imu_usb.rules
```

修改后重新执行：

```bash
sudo bash bind_usb.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

再重新插拔 IMU，检查：

```bash
ll /dev/imu_usb
```

本机这次实际排查结果：

- 厂家默认 `imu_usb.rules` 写的是：
  - `idVendor=10c4`
  - `idProduct=ea60`
- 你当前电脑识别到的 IMU 实际是：
  - `idVendor=1a86`
  - `idProduct=7523`
- `dmesg` 已确认它被挂成：
  - `/dev/ttyUSB0`

也就是说，这次 `/dev/imu_usb` 没有生成，不是 IMU 没识别到，而是**udev 规则的 USB ID 和当前这块 IMU 实际 ID 不匹配**。

因此当前机器上应把规则改成类似：

```text
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="imu_usb"
```

然后重新执行：

```bash
sudo bash bind_usb.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

最后重新插拔 IMU，再检查：

```bash
ll /dev/imu_usb
```

### 2.6 如果还是不行，先走手动串口号

```bash
ls /dev/ttyUSB*
```

建议这样确认：

1. 不插 IMU 时查一次
2. 插上 IMU 后再查一次
3. 新出现的那个就是 IMU 串口

如果识别出来的是 `/dev/ttyUSB1`，后面 launch 就改成 `/dev/ttyUSB1`。

本机当前实际识别结果是：

```text
/dev/ttyUSB0
```

如果只是临时权限问题，可短期排障用：

```bash
sudo chmod 777 /dev/ttyUSB1
```

说明：

- 这只是临时排障手段
- 常规使用仍然建议优先把 `udev` 规则配置好

当前这台电脑还需要额外注意一件事：

- `/dev/ttyUSB0` 当前权限是 `root:dialout`
- 当前用户 `a` 不在 `dialout` 组里

也就是说，直接打开串口时，可能会报：

```text
Permission denied
Serial port opening failure
```

如果你只是想先把驱动跑起来，当前最短临时处理方式是：

```bash
sudo chmod 666 /dev/ttyUSB0
```

长期做法是：

```bash
sudo usermod -aG dialout $USER
```

然后重新登录。

### 2.6A 当前机器的最短 bring-up 路线

当前机器已经确认：

- ROS 版本：`noetic`
- IMU 实际串口：`/dev/ttyUSB0`
- 厂家脚本默认波特率：`9600`
- 厂家脚本本身就是一个 Python ROS 节点
- 节点参数只需要：
  - `~port`
  - `~baud`
- 厂家脚本会把 `header.frame_id` 直接写成 `base_link`
- 当前实际包位置已经改成：
  - `/home/a/scout_ws/src/wit_ros_imu`
- 当前工作区环境文件存在：
  - `/home/a/scout_ws/devel/setup.bash`

所以，想最快看到 `/imu/data`，不需要先改 launch，也不需要先 `catkin_make`。  
可以直接在 `scout_ws` 环境下用 `rosrun` 跑厂家脚本，再把 `wit/imu` remap 到 `/imu/data`。

### 2.6B 直接运行驱动节点（不带 RViz）

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
roscore
```

终端 2：

```bash
sudo chmod 666 /dev/ttyUSB0
chmod +x /home/a/scout_ws/src/wit_ros_imu/scripts/wit_normal_ros.py
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
rosrun wit_ros_imu wit_normal_ros.py _port:=/dev/ttyUSB0 _baud:=9600 wit/imu:=/imu/data
```

终端 3：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
rostopic list | grep imu
rostopic echo -n1 /imu/data
rostopic hz /imu/data
rostopic echo -n1 /wit/mag
```

说明：

- 现在厂家包已经放进了 `scout_ws/src`，所以优先直接 `source /home/a/scout_ws/devel/setup.bash`
- 厂家脚本默认发布的是 `wit/imu`，这里通过命令行 remap 到 `/imu/data`
- 当前机器实测还需要先执行：
  - `chmod +x /home/a/scout_ws/src/wit_ros_imu/scripts/wit_normal_ros.py`
  - 因为解压后的脚本默认没有可执行位，`rosrun` 会报：
    - `Couldn't find executable named wit_normal_ros.py`
- 如果 `source /home/a/scout_ws/devel/setup.bash` 后仍然报：
  - `package 'wit_ros_imu' not found`
  - 则手动补一条：
    - `export ROS_PACKAGE_PATH=/home/a/scout_ws/src:$ROS_PACKAGE_PATH`
- 这条路线的目的只是先确认驱动和串口正常，不处理 RViz
- 如果这条路线能成功，再决定是否继续按厂家 PDF 的 `ros1Imu_ws + roslaunch` 方式整理

### 2.6C 本机实际 bring-up 结果（2026-03-16）

本机最终采用的是：

- 不等 `/dev/imu_usb`
- 直接使用 `/dev/ttyUSB0`
- 用 `rosrun` 启动厂家脚本
- 把 `wit/imu` remap 到 `/imu/data`

实际启动命令：

```bash
sudo chmod 666 /dev/ttyUSB0
chmod +x /home/a/scout_ws/src/wit_ros_imu/scripts/wit_normal_ros.py
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
rosrun wit_ros_imu wit_normal_ros.py _port:=/dev/ttyUSB0 _baud:=9600 wit/imu:=/imu/data
```

节点启动输出：

```text
IMU Type: Normal Port:/dev/ttyUSB0 baud:9600
Serial port opened successfully...
```

实际话题结果：

- `rostopic list` 中已看到：
  - `/imu/data`
  - `/wit/mag`
- `rostopic hz /imu/data` 实测频率约：
  - `10.018 Hz`

实际 `rostopic echo -n1 /imu/data` 关键字段：

- `header.frame_id = "base_link"`
- `angular_velocity.z = 0.0`
- `linear_acceleration.x = 0.153125`
- `linear_acceleration.y = 0.0478515625`
- `linear_acceleration.z = 9.8287109375`

当前可直接得出的结论：

1. 电脑端厂家驱动 bring-up 已成功
2. `/imu/data` 已正常发布
3. `/wit/mag` 也在发布
4. 当前静止样本里 `linear_acceleration.z` 约为 `9.83`
5. 这说明该驱动当前发布的加速度数据**包含重力分量**

因此，这一步虽然已经满足“电脑端 bring-up 成功”，但还**不能**直接推出：

- IMU 已可无条件直接接入 `scout_local_planner`
- `linear_acceleration.y` 已可直接作为 planner 的 lateral accel 输入

原因是：

- 厂家脚本把 `frame_id` 直接写成了 `base_link`
- 这个 `frame_id` 只能说明消息里这样填了，**不能单独证明物理安装方向一定正确**
- 同时当前静止输出已显示加速度里带重力分量

所以，进入上位机阶段后，仍然必须继续做：

- 轴向核对
- 符号核对
- 静止输出复验
- 是否需要预处理节点做重力补偿的判断

### 2.6D 工控机端建议使用独立 launch

电脑端这次是为了尽快 bring-up，采用了直接 `rosrun` 的最短路径。  
但到了工控机端，不建议继续长期手敲 `rosrun`，而是改用独立 launch。

当前仓库里已新增：

- [imu_only.launch](/home/a/scout_ws/src/wit_ros_imu/launch/imu_only.launch)
- [scout_imu.launch](/home/a/scout_ws/src/scout_ros/scout_bringup/launch/scout_imu.launch)

推荐工控机端使用：

```bash
roslaunch scout_bringup scout_imu.launch
```

如果 `udev` 绑定还没修好，可临时指定串口：

```bash
roslaunch scout_bringup scout_imu.launch port:=/dev/ttyUSB0
```

当前这个独立 launch 相比厂家 `rviz_and_imu.launch` 的差异是：

- 不启动 RViz
- 可以单独指定 `port`
- 可以单独指定 `baud`
- 可以单独指定 `frame_id`
- 默认仍发布到：
  - `/imu/data`
  - `/wit/mag`

当前脚本也已改成支持参数：

```text
~frame_id
```

因此工控机端如果你后续决定不用 `base_link`，也可以这样起：

```bash
roslaunch scout_bringup scout_imu.launch frame_id:=imu_link
```

注意：

- 这只会改变消息头里的 `frame_id`
- 不会自动帮你做 TF 旋转
- 也不会自动做重力补偿

### 2.7 建独立工作空间

不要先把厂家包混进你当前小车工程工作空间，先单独验证。

```bash
mkdir -p ~/ros1Imu_ws/src
cp -r ~/wit_ros_imu ~/ros1Imu_ws/src/
cd ~/ros1Imu_ws
catkin_make
```

### 2.8 给 Python 脚本权限

```bash
find ~/ros1Imu_ws/src -name "*.py" -exec chmod +x {} \;
```

### 2.9 source 工作空间

先一次性 source 即可，不必照抄厂家 PDF 里的 `.zshrc`。

你当前环境是 `bash`，所以优先用：

```bash
source ~/ros1Imu_ws/devel/setup.bash
```

确认包能找到：

```bash
rospack find wit_ros_imu
```

### 2.10 修改 launch 里的串口和波特率

```bash
cd ~/ros1Imu_ws/src/wit_ros_imu/launch
vim rviz_and_imu.launch
```

重点检查：

```xml
<param name="port" value="/dev/imu_usb"/>
<param name="baud" value="9600"/>
```

如果自动绑定没成功，就改成实际串口，例如：

```xml
<param name="port" value="/dev/ttyUSB1"/>
```

如果你在上位机工具里改过波特率，这里也必须同步修改。

### 2.11 启动厂家驱动

```bash
source ~/ros1Imu_ws/devel/setup.bash
roslaunch wit_ros_imu rviz_and_imu.launch
```

如果你跑的是厂家自带机器人镜像，并提示 APP 占用，可先执行：

```bash
sudo systemctl stop start_app_node.service
```

注意：

- 要在桌面或 VNC 环境打开
- 不要指望在纯 SSH 终端里让 RViz 正常显示

### 2.12 验证话题

新开终端，执行：

```bash
source ~/ros1Imu_ws/devel/setup.bash
rostopic list | grep imu
rostopic echo -n1 /imu/data
rostopic hz /imu/data
rostopic echo -n1 /wit/mag
```

到这里，电脑端厂家层 bring-up 就完成了。

---

## 3. 上位机（工控机）阶段的目标

上位机阶段不是简单重复电脑端 bring-up，而是要补齐**实机可用性检查**。

上位机阶段的完成标准比电脑端多四项：

1. `/imu/data` 持续发布
2. `header.frame_id` 已确认
3. 静止放平时输出行为已确认
4. `axis / sign` 已和车体坐标约定核对

当前项目建议采用 `base_link` 常用约定：

- `x` 向前
- `y` 向左
- `z` 向上

也就是 REP-103 常见约定。

---

## 4. 上位机 bring-up 流程

上位机上先重复一遍第 2 节的厂家层流程，目标同样是先让 `/imu/data` 正常出来。  
工作空间路径可按上位机实际环境调整，但建议仍然先独立建，例如：

```bash
mkdir -p ~/ros1Imu_ws/src
```

不要一上来就把厂家包塞进 `scout_ws` 里混编。

当上位机已经满足：

```bash
rostopic echo -n1 /imu/data
```

以后，再做下面四项核验。

---

## 5. 上位机必须核验的四项

### 5.1 `frame_id`

先看消息头：

```bash
rostopic echo -n1 /imu/data/header
```

重点记录：

- `frame_id` 是什么
- 是否为空字符串
- 是否和你预期的 IMU 安装坐标系一致

如果系统里已经发布了 IMU 的 TF，再查：

```bash
rosrun tf tf_echo base_link imu_link
```

如果实际 frame 不是 `imu_link`，把命令里的 frame 替换成真实名称。

注意：

- 当前 `scout_local_planner` 不会根据 `frame_id` 自动旋转 IMU 数据
- 所以 `frame_id` 不是拿来“看一眼就完了”，而是要确认实际安装方向是否已经和 `base_link` 对齐

如果 `frame_id` 正常，但 IMU 实际装歪了，当前代码仍然会直接吃错轴数据。

### 5.2 静止输出

让小车停稳、放平、静止 5 到 10 秒，然后观察：

```bash
rostopic echo /imu/data
```

至少检查这些字段：

- `angular_velocity.z`
- `linear_acceleration.y`
- `linear_acceleration.z`

期望现象：

- `angular_velocity.z` 应该接近 0
- 如果车体放平且 IMU 安装正确，`linear_acceleration.y` 应接近 0
- 某一轴如果稳定在约 `+9.8` 或 `-9.8`，说明该驱动很可能发布的是**含重力分量**的数据

这一步的判断很关键：

- 如果 `linear_acceleration.y` 在静止放平时就长期不接近 0，当前不适合直接拿来替换 lateral accel
- 如果 `linear_acceleration` 含明显重力分量，后续接 planner 前要先明确是否需要预处理节点做重力补偿

### 5.3 角速度 `z` 的符号

原地缓慢左转和右转，观察：

```bash
rostopic echo /imu/data/angular_velocity/z
```

按 `x` 前、`y` 左、`z` 上的约定：

- 逆时针左转，`angular_velocity.z` 应为正
- 顺时针右转，`angular_velocity.z` 应为负

如果符号相反，当前不能直接打开：

```text
slosh_use_imu_yaw_rate:=true
slosh_use_imu_alpha_z:=true
```

因为后者是由 `z` 角速度差分得到的，符号会一起错。

### 5.4 横向加速度 `y` 的轴向和符号

这一项不要在纯静止状态下判断，要做低风险的小动作测试。

建议做法：

1. 低速直行
2. 做小幅左转
3. 做小幅右转
4. 记录 `linear_acceleration.y`

观察命令：

```bash
rostopic echo /imu/data/linear_acceleration/y
```

你要确认的是：

- 左右两种动作时，`y` 输出会稳定反号
- 这个反号关系和你定义的 `base_link` 约定一致

说明：

- 不同厂家驱动可能发布的是原始加速度、补偿后的线加速度，或带滤波值
- 因此这里不要凭想象直接认定正负号，应该以实际测试结果为准
- 但至少要保证：同一类动作多次重复时，符号关系稳定一致

如果你发现：

- 左右动作的符号关系不稳定
- 静止时 `y` 漂得很大
- 车体放平时 `y` 明显带重力偏置

那就不要直接打开：

```text
slosh_use_imu_lateral_accel:=true
```

---

## 6. 上位机阶段的建议验收命令

建议按下面顺序执行并记录结果：

```bash
rostopic list | grep imu
rostopic hz /imu/data
rostopic echo -n1 /imu/data/header
rostopic echo -n5 /imu/data
rostopic echo /imu/data/angular_velocity/z
rostopic echo /imu/data/linear_acceleration/y
```

如果有 TF，再补：

```bash
rosrun tf tf_echo base_link imu_link
```

---

## 7. 如果核验不过，当前项目里的处理原则

如果上位机阶段发现任一问题：

- 轴向不对
- 符号不对
- `frame_id` 不清楚
- 静止输出不合理

当前建议处理顺序是：

1. 先调整 IMU 物理安装方向，使其尽量和 `base_link` 一致
2. 若不能靠安装解决，再增加一个 IMU 预处理节点
3. 让预处理节点输出一个“已转正、已确认”的新话题，例如 `/imu/data_scout`
4. 后续再把 `slosh_imu_topic` 指向这个新话题

在这之前，保持：

```text
slosh_use_imu_lateral_accel:=false
slosh_use_imu_yaw_rate:=false
slosh_use_imu_alpha_z:=false
```

---

## 8. 后续接入 `scout_local_planner` 的前置条件

只有在下面条件都满足后，才进入阶段 7 的 planner 接入验证：

1. `/imu/data` 已在实机稳定发布
2. `frame_id` 已记录清楚
3. 轴向和符号已确认
4. 静止输出已确认可用

到那时再用下面这种方式做最小验证：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_use_imu_lateral_accel:=true \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_alpha_z:=true \
  slosh_imu_topic:=/imu/data
```

注意：

- 阶段 7 验证时，优先只切 IMU 参数
- 不要同时再改一组 governor 参数

---

## 9. 一页版执行清单

### 9.1 电脑端

```bash
echo $ROS_DISTRO

sudo apt-get update
sudo apt-get install ros-noetic-imu-tools ros-noetic-rviz-imu-plugin
pip3 install pyserial
sudo apt-get install ros-$ROS_DISTRO-serial

cd ~
unzip /home/a/scout_ws/src/ros1_imu.zip

cd ~/wit_ros_imu
sudo bash bind_usb.sh
# 重新插拔 IMU
ll /dev/imu_usb

# 如果 /dev/imu_usb 没出来，检查实际 USB ID
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
lsusb
dmesg | tail -n 50

# 本机当前观测到的实际 ID 是 1a86:7523，对应 /dev/ttyUSB0
# 需要把 imu_usb.rules 改成匹配 1a86:7523，再重新 bind + reload udev
# 如果暂时不改 rules，也可以先在 launch 里直接使用 /dev/ttyUSB0

# 当前用户不在 dialout 组，先临时放开串口权限
sudo chmod 666 /dev/ttyUSB0

# 最短 bring-up：先不改 launch，不带 RViz，直接跑驱动
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
roscore

# 新开终端
sudo chmod 666 /dev/ttyUSB0
chmod +x /home/a/scout_ws/src/wit_ros_imu/scripts/wit_normal_ros.py
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
rosrun wit_ros_imu wit_normal_ros.py _port:=/dev/ttyUSB0 _baud:=9600 wit/imu:=/imu/data

# 再新开终端验证
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
rostopic list | grep imu
rostopic echo -n1 /imu/data
rostopic hz /imu/data
rostopic echo -n1 /wit/mag

# 如果最短 bring-up 已成功，再按厂家工作空间方式整理
mkdir -p ~/ros1Imu_ws/src
cp -r ~/wit_ros_imu ~/ros1Imu_ws/src/
cd ~/ros1Imu_ws
catkin_make
find ~/ros1Imu_ws/src -name "*.py" -exec chmod +x {} \;
source ~/ros1Imu_ws/devel/setup.bash

cd ~/ros1Imu_ws/src/wit_ros_imu/launch
vim rviz_and_imu.launch
# 检查 port 和 baud

source ~/ros1Imu_ws/devel/setup.bash
roslaunch wit_ros_imu rviz_and_imu.launch

rostopic list | grep imu
rostopic echo -n1 /imu/data
rostopic hz /imu/data
```

### 9.2 上位机

```bash
rostopic list | grep imu
rostopic hz /imu/data
rostopic echo -n1 /imu/data/header
rostopic echo -n5 /imu/data
rostopic echo /imu/data/angular_velocity/z
rostopic echo /imu/data/linear_acceleration/y
```

需要确认四件事：

- `frame_id`
- 静止输出
- `angular_velocity.z` 符号
- `linear_acceleration.y` 轴向与符号

### 9.3 工控机实测结果（2026-03-16）

本次工控机实际已完成：

1. 依赖确认：`pyserial` 已安装（`Requirement already satisfied`）
2. `udev` 规则已安装到系统：
  - `/etc/udev/rules.d/imu_usb.rules`
3. 规则已重载：
  - `sudo udevadm control --reload-rules`
  - `sudo udevadm trigger`
4. 软链接已生效：
  - `/dev/imu_usb -> ttyUSB0`
5. 实际串口权限：
  - `/dev/ttyUSB0` 为 `crw-rw-rw-`
6. `scout_imu.launch` 已可启动并发布：
  - `/imu/data`
  - `/wit/mag`

本次实测 `rostopic echo -n1 /imu/data` 关键字段：

- `header.frame_id: "base_link"`
- `angular_velocity.z: 0.0`
- `linear_acceleration.x: 0.08134765625000001`
- `linear_acceleration.y: 0.0765625`
- `linear_acceleration.z: 9.838281250000001`

结论：

- 厂家驱动在工控机端已可稳定运行
- `linear_acceleration.z` 约 `9.84`，当前数据仍表现为包含重力分量

### 9.4 关于 `(.venv)` 提示

如果终端前缀出现 `(.venv)`，表示当前 shell 激活了 Python 虚拟环境。

- 对本 IMU 节点（ROS Noetic + `python3`）来说，不是必须条件
- 退出方式：

```bash
deactivate
```

- 若 `deactivate` 提示不存在，说明当前终端其实没有激活 venv

建议：工控机日常运行 IMU 驱动时，优先使用系统 Python（`/usr/bin/python3`）即可。

### 9.5 当前配置是否长期生效

当前配置“基本可长期生效”，前提是硬件接法不变：

1. `udev` 规则文件放在 `/etc/udev/rules.d/`，重启后仍在
2. `scout_imu.launch` 默认使用 `/dev/imu_usb`，不受 `ttyUSBx` 编号漂移影响
3. 脚本 shebang 已改为 `python3`，Noetic 环境可直接运行

但有两点仍需注意：

- `sudo usermod -aG dialout $USER` 需要“重新登录”后才对当前用户会话生效
- 若后续接入其它同类 USB 串口设备（同为 `1a86:7523`），建议再加更细粒度规则（按物理端口或序列号）以避免歧义
