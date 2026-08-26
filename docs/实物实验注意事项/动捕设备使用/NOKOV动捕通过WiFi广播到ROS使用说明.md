# NOKOV/XINGYING 动捕通过 Wi‑Fi 向 ROS 广播数据的使用说明

> 适用环境：NOKOV/XINGYING 3.x、Windows 动捕主机、Ubuntu 20.04、ROS1 Noetic。
> 文档目标：让另一台连接同一 Wi‑Fi 的 Ubuntu 电脑，通过 VRPN 接收 XINGYING 中的刚体或 Marker 数据。
> 现场验证日期：2026-07-29。现场曾以 `192.168.203.85:3883`、刚体 `Tracker0` 成功接收约 90 Hz 的位姿数据。IP、刚体名和实际频率都可能随现场变化。

## 1. 先理解数据链路

```text
NOKOV 相机
    ↓
Windows 动捕主机上的 XINGYING
    ↓  在“数据广播”面板开启 VRPN，并绑定 Windows 的 Wi‑Fi 地址
Windows Wi‑Fi IPv4 : VRPN 端口
    ↓  同一 Wi‑Fi 局域网
Ubuntu 上的 vrpn_client_ros
    ↓
/vrpn_client_node/<刚体名>/pose
```

这里常说的“Wi‑Fi 广播”，实质上是：

1. XINGYING 在 Windows 主机上运行 VRPN 服务；
2. Ubuntu 通过 Windows 的 Wi‑Fi IPv4 地址连接这个服务；
3. VRPN 自动发现 XINGYING 发布的刚体或 Marker 名称，并在 ROS 中创建相应话题。

接收端不是连接 `255.255.255.255` 这样的广播地址，也不能把 Windows 上的服务误写成 `127.0.0.1`。只要 XINGYING 不在 Ubuntu 本机运行，`127.0.0.1` 就一定不对。

## 2. 需要提前确认的三个值

以下是曾经成功使用的现场示例：

```text
Windows 动捕主机 Wi‑Fi IPv4：192.168.203.85
VRPN 端口：                  3883
XINGYING 刚体名称：          Tracker0
```

分享给别人时，应让对方替换成自己的实际值：

```bash
export MOCAP_HOST=192.168.203.85
export MOCAP_PORT=3883
export MOCAP_TRACKER=Tracker0
```

- `MOCAP_HOST`：运行 XINGYING 的 Windows 主机在当前 Wi‑Fi 上的 IPv4；
- `MOCAP_PORT`：XINGYING/VRPN 的监听端口，常用默认值为 `3883`；
- `MOCAP_TRACKER`：XINGYING 中的 Markerset 或刚体名称，严格区分大小写。

## 3. Windows 与 XINGYING 端设置

### 3.1 让两台电脑加入同一个 Wi‑Fi

Windows 动捕主机和 Ubuntu 接收机必须位于可互相访问的局域网中。

不建议使用带“客户端隔离/AP isolation”的访客网络。访客网络即使给两台电脑分配了同一网段的地址，也可能禁止设备之间互访。

如果 Windows 同时连接了：

- NOKOV 相机使用的有线采集网卡；
- 与 Ubuntu 通信的 Wi‑Fi 网卡；

那么 XINGYING 的 VRPN 广播应绑定到 Ubuntu 能访问的 Wi‑Fi 地址，而不是盲目沿用相机采集网卡的地址。

### 3.2 查询 Windows 的 Wi‑Fi IPv4

在 Windows 的命令提示符中执行：

```bat
ipconfig
```

找到“无线局域网适配器 WLAN”下的 IPv4 地址。例如：

```text
IPv4 地址 . . . . . . . . . . . . : 192.168.203.85
```

不要使用：

- `127.0.0.1`；
- 虚拟机、VPN、蓝牙网卡的地址；
- Ubuntu 无法路由到的另一块网卡地址；
- 断开 Wi‑Fi 后遗留的旧地址。

Wi‑Fi 通过 DHCP 分配地址时，Windows 重连后地址可能变化。长期使用建议在路由器中做 DHCP 地址保留，或每次实验前重新运行 `ipconfig`。

### 3.3 在 XINGYING 中建立刚体

1. 确认相机已经完成标定，实时模式能够稳定看到 Marker；
2. 创建 Markerset/刚体；
3. 使用简单、固定、无空格的英文名称，例如 `Tracker0` 或 `Scout`；
4. 确认刚体在 XINGYING 中持续被解算，位置和姿态会随运动变化。

如果只发送单个 Marker，ROS 通常只能得到点位置；需要完整位置和姿态时，应发送“刚体”类型。

### 3.4 开启数据广播和 VRPN

XINGYING 版本不同，面板名称可能略有区别。一般操作如下：

1. 暂停播放；
2. 打开“视图 → 数据广播”面板；
3. 在“网卡地址”下拉框选择 Windows 的 Wi‑Fi IPv4，例如 `192.168.203.85`；
4. 选择需要发送的数据类型，推荐先只勾选“刚体”；
5. 将单位设为“米”，便于直接用于 ROS；
6. 确认 VRPN 端口为 `3883`，或记录实际端口；
7. 勾选“启用 VRPN”；
8. 点击播放，保持 XINGYING 处于实时运行状态。

部分 XINGYING 版本或厂家文档会要求同时开启“SDK”和“VRPN”。第一次配置时，如果界面中存在独立的 SDK 总开关，建议同时打开；最终 ROS 数据仍通过 VRPN 接收。

修改数据类型、单位、轴反转、偏移或网卡地址时，建议先暂停播放并关闭 VRPN，改完后重新开启 VRPN 和播放。

### 3.5 检查 Windows 是否监听端口

在 Windows 命令提示符中执行：

```bat
netstat -ano | findstr :3883
```

若 VRPN 已正常启动，应看到对应端口处于监听或已建立连接状态。如果完全没有结果，优先检查：

- XINGYING 是否正在播放；
- 是否勾选“启用 VRPN”；
- “网卡地址”是否选中了 Wi‑Fi 地址；
- VRPN 插件是否已经正确安装和授权；
- 实际端口是否不是 `3883`。

厂家提供 `NokovVrpnClient.exe` 时，也可以先在 Windows 本机运行它验证 VRPN。Windows 客户端能收到数据，说明 XINGYING 端基本配置正确；Ubuntu 仍收不到时，再集中排查 Wi‑Fi、路由和防火墙。

### 3.6 Windows 防火墙

推荐在 Windows Defender 防火墙中允许 XINGYING/VRPN 程序通过“专用网络”。如果无法按程序放行，可由管理员按现场安全要求放行 VRPN 使用的 TCP/UDP 端口。

不要长期关闭整台机器的防火墙。临时关闭防火墙只适合作为诊断手段：如果关闭后立刻能连接，就应恢复防火墙并创建精确规则。

## 4. Ubuntu 端检查 Wi‑Fi 网络

### 4.1 确认本机地址和路由

```bash
ip -br -4 address
ip route
ip route get "${MOCAP_HOST}"
```

如果 Windows 是 `192.168.203.85`，Ubuntu 通常也应有类似 `192.168.203.xxx/24` 的 Wi‑Fi 地址。最后一条命令应显示数据会从正确的无线网卡发出。

“同一个 Wi‑Fi 名称”不总是等于“可以互访”。如果地址看似正确但无法访问，还要检查：

- 是否接入访客网络；
- 路由器是否启用了客户端隔离；
- Ubuntu 是否开启了会抢占路由的 VPN；
- Windows 防火墙是否阻止连接。

### 4.2 测试主机连通性

```bash
ping -c 3 "${MOCAP_HOST}"
```

现场成功示例：

```bash
ping -c 3 192.168.203.85
```

Windows 可能只禁止 ICMP，因此“ping 不通”不一定代表 VRPN 一定不通。还应直接测试 VRPN TCP 端口：

```bash
nc -zv -w 2 "${MOCAP_HOST}" "${MOCAP_PORT}"
```

成功时会看到类似：

```text
Connection to 192.168.203.85 3883 port [tcp/*] succeeded!
```

如果系统没有 `nc`：

```bash
sudo apt-get update
sudo apt-get install -y netcat-openbsd
```

判断方法：

| `ping` | `nc 3883` | 含义 |
|---|---|---|
| 成功 | 成功 | 网络和 VRPN 监听端口基本正常 |
| 失败 | 成功 | Windows 可能只禁止 ICMP，可以继续启动 ROS 客户端 |
| 成功 | 失败 | VRPN 未监听、绑定错网卡、端口错误或防火墙拦截 |
| 失败 | 失败 | 优先检查 Wi‑Fi、网段、客户端隔离、路由和防火墙 |

## 5. ROS1 Noetic 接收原始 VRPN 数据

### 5.1 一次性安装依赖

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-vrpn-client-ros
```

检查：

```bash
source /opt/ros/noetic/setup.bash
rospack find vrpn_client_ros
```

正常时会输出类似：

```text
/opt/ros/noetic/share/vrpn_client_ros
```

### 5.2 启动 VRPN 客户端

终端 1：

```bash
source /opt/ros/noetic/setup.bash

export MOCAP_HOST=192.168.203.85
roslaunch vrpn_client_ros sample.launch server:="${MOCAP_HOST}"
```

现场成功连接时出现过以下日志：

```text
Connecting to VRPN server at 192.168.203.85:3883
Connection established
Found new sender: Tracker0
Creating new tracker Tracker0
```

通过标准不只是 `Connection established`。至少还应看到 `Found new sender`，并在 ROS 中出现刚体话题。

本机 ROS Noetic 自带的 `sample.launch` 只接受 `server` 参数，端口在文件中固定为 `3883`。因此不要直接照抄下面这种命令：

```text
roslaunch vrpn_client_ros sample.launch server:=... port:=3883
```

否则可能报：

```text
RLException: unused args [port]
```

如果现场 VRPN 不是 `3883`，应先查看所安装的 `sample.launch` 是否支持端口参数；不支持时需要复制一份 launch 并修改其中的 `port` ROS 参数。

### 5.3 自动发现刚体名称和话题

终端 2：

```bash
source /opt/ros/noetic/setup.bash
rostopic list | grep '^/vrpn_client_node/' | sort
```

刚体为 `Tracker0` 时，通常会出现：

```text
/vrpn_client_node/Tracker0/accel
/vrpn_client_node/Tracker0/pose
/vrpn_client_node/Tracker0/twist
```

其中最重要的是：

```text
/vrpn_client_node/Tracker0/pose
```

不要只凭记忆猜刚体名。以 `rostopic list` 自动发现的名称为准，并注意大小写。

### 5.4 检查数据内容和频率

```bash
export MOCAP_TRACKER=Tracker0

rostopic echo -n 1 "/vrpn_client_node/${MOCAP_TRACKER}/pose"
rostopic hz "/vrpn_client_node/${MOCAP_TRACKER}/pose"
```

`pose` 消息应包含：

- `header.stamp`：时间戳；
- `header.frame_id`：常见为 `world`；
- `position.x/y/z`：位置；
- `orientation.x/y/z/w`：四元数姿态。

如果 XINGYING 的单位设置为“米”，ROS 的位置也应按米解释。现场 `Tracker0` 曾稳定在约 90 Hz，但实际频率取决于动捕配置，不能把 90 Hz 当作所有设备的固定要求。

最后做一次人工运动检查：

1. 保持刚体静止，观察位置是否稳定；
2. 缓慢移动刚体，确认位置连续变化；
3. 缓慢旋转刚体，确认四元数连续变化；
4. 暂时遮挡部分 Marker，确认刚体恢复后不会跳到错误目标。

## 6. 在 `scout_ws` 中桥接为 `/mocap/*` 话题

这一节只适用于拥有 `/home/geist/scout_ws` 项目的机器。普通接收者只需要上一节的原始 VRPN 话题即可。

由于本机 `vrpn_client_ros/sample.launch` 不接受外部 `port` 参数，现场验证过的可靠方式是分两个终端启动。

### 6.1 终端 1：启动原始 VRPN 客户端

```bash
source /opt/ros/noetic/setup.bash
roslaunch vrpn_client_ros sample.launch server:=192.168.203.85
```

### 6.2 终端 2：只启动项目监控桥

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

roslaunch nokov_mocap_monitor nokov_monitor.launch \
  start_vrpn:=false \
  server:=192.168.203.85 \
  tracker:=Tracker0
```

桥接后会增加：

```text
/mocap/scout_pose
/mocap/scout_odom
/mocap/scout_path
/mocap/status
```

检查：

```bash
rostopic list | grep -E 'vrpn|mocap'
rostopic echo -n 1 /mocap/scout_pose
rostopic echo -n 1 /mocap/status
rostopic hz /mocap/scout_pose
```

正常状态类似：

```text
OK tracker=Tracker0 input=/vrpn_client_node/Tracker0/pose ...
```

也可以运行项目自检脚本：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

NOKOV_SERVER=192.168.203.85 MOCAP_TRACKER=Tracker0 \
  rosrun nokov_mocap_monitor check_nokov_env.sh
```

项目中的 `/mocap/*` 默认只用于监控、录包和离线真值分析，不应直接替换 `/odom`，也不应在未经设计和验证时接入控制闭环。

## 7. 录制动捕数据

### 7.1 只录原始 VRPN

```bash
export MOCAP_TRACKER=Tracker0
mkdir -p ~/mocap_bags

rosbag record \
  -O ~/mocap_bags/mocap_$(date +%Y%m%d_%H%M%S).bag \
  "/vrpn_client_node/${MOCAP_TRACKER}/pose" \
  "/vrpn_client_node/${MOCAP_TRACKER}/twist" \
  "/vrpn_client_node/${MOCAP_TRACKER}/accel"
```

### 7.2 同时录制项目桥接话题

```bash
export MOCAP_TRACKER=Tracker0
mkdir -p ~/mocap_bags

rosbag record \
  -O ~/mocap_bags/mocap_full_$(date +%Y%m%d_%H%M%S).bag \
  "/vrpn_client_node/${MOCAP_TRACKER}/pose" \
  "/vrpn_client_node/${MOCAP_TRACKER}/twist" \
  "/vrpn_client_node/${MOCAP_TRACKER}/accel" \
  /mocap/scout_pose \
  /mocap/scout_odom \
  /mocap/status \
  /tf \
  /tf_static
```

使用 `Ctrl+C` 正常停止录制，然后检查：

```bash
rosbag info ~/mocap_bags/mocap_20260819_120000.bag
```

上面的文件名只是示例，请替换成刚刚实际生成的 bag 名称。`~` 表示当前用户的主目录，因此复制给其他用户时不需要修改用户名。

长时间录制时可以不录 `/mocap/scout_path`，因为 `Path` 会不断累积历史点，可能让 bag 体积快速增大。

## 8. 常见问题排查

### 8.1 把服务器写成了 `127.0.0.1`

现象：客户端连接到了本机，但看不到 Windows 中的真实刚体。

原因：`127.0.0.1` 永远指当前 Ubuntu 本机。只有 VRPN 服务也运行在这台 Ubuntu 上时才应使用它。

处理：改成 `ipconfig` 查到的 Windows Wi‑Fi IPv4。

### 8.2 `ping` 和 `nc` 都不通

依次检查：

1. 两台机器是否确实连接同一个可互访的 Wi‑Fi；
2. 是否使用访客网络或开启了 AP isolation；
3. Windows IP 是否已因 DHCP 变化；
4. XINGYING 的“网卡地址”是否选中 Wi‑Fi 地址；
5. Ubuntu 的 VPN 或静态路由是否把流量导向错误网卡；
6. Windows 防火墙是否拦截。

### 8.3 `ping` 通，但 `3883` 端口不通

优先检查：

- XINGYING 是否正在播放；
- 是否开启 VRPN；
- VRPN 端口是否确实为 `3883`；
- VRPN 是否绑定到了另一块网卡；
- 防火墙是否允许 XINGYING/VRPN。

### 8.4 显示 `Connection established`，但没有刚体话题

`Connection established` 只代表连到了 VRPN 服务，不代表已经收到有效 sender。继续检查：

- 日志中是否出现 `Found new sender: <名称>`；
- XINGYING 中是否已经创建并稳定解算刚体；
- 数据广播是否勾选“刚体”；
- XINGYING 是否处于播放状态；
- Marker 是否被遮挡；
- 用 `rostopic list` 查看实际名称，不要假设一定叫 `Tracker0`。

### 8.5 话题存在，但数据停止或 `/mocap/status` 为 `STALE`

检查：

```bash
rostopic hz /vrpn_client_node/Tracker0/pose
```

同时观察：

- XINGYING 中刚体是否丢失；
- Wi‑Fi 信号是否变差；
- Windows 是否休眠、切换网卡或重连 Wi‑Fi；
- 防火墙/安全软件是否中断数据；
- XINGYING 是否被暂停。

### 8.6 出现 `unused args [port]`

本机安装的 ROS Noetic `sample.launch` 端口固定为 `3883`，只接受 `server` 参数。

使用：

```bash
roslaunch vrpn_client_ros sample.launch server:=192.168.203.85
```

项目监控桥另开终端，并设置 `start_vrpn:=false`。

### 8.7 Wi‑Fi 下频率不稳定或延迟较大

可以尝试：

- 使用 5 GHz Wi‑Fi，并让接入点靠近实验区域；
- 避免拥挤信道和访客网络；
- 禁止 Windows/Ubuntu 在实验中自动切换网络；
- 关闭会改变路由的 VPN；
- 在路由器中固定 Windows 地址；
- 对时延、丢包和时间同步敏感的正式标定优先使用有线连接。

Wi‑Fi 可以方便地传输监控和真值数据，但不能因为 `rostopic hz` 平均频率正常，就假设每一帧的网络时延都固定。

## 9. 时间戳和坐标系注意事项

ROS Noetic 标准 `sample.launch` 通常配置为：

```text
use_server_time: false
frame_id: world
```

这表示消息常使用接收端 ROS 时间，仍包含 Wi‑Fi 传输和调度延迟。如果动捕要与 IMU、相机或控制命令做精确时延标定，应：

- 保留原始 `/vrpn_client_node/<刚体>/pose`；
- 同时录制其他传感器的原始时间戳；
- 不把网络时延假设为严格常数；
- 如需使用服务端时间，应先完成 Windows 与 Ubuntu 的时钟同步并验证时间戳语义。

XINGYING 中的轴反转、单位和位置偏移会直接改变 ROS 收到的数据。分享数据前应记录：

- VRPN 单位；
- 坐标轴方向；
- 动捕世界原点；
- 刚体原点和 Marker 布置；
- 是否使用位置偏移；
- 刚体名称和 VRPN 端口。

## 10. 一页式现场清单

### Windows/XINGYING

- [ ] Windows 与 Ubuntu 接入同一个可互访 Wi‑Fi；
- [ ] `ipconfig` 记录 Windows Wi‑Fi IPv4；
- [ ] XINGYING 中刚体稳定可见，名称已记录；
- [ ] 数据广播的“网卡地址”选择 Windows Wi‑Fi IPv4；
- [ ] 数据类型选择“刚体”，单位选择“米”；
- [ ] 开启 VRPN，必要时同时开启 SDK；
- [ ] XINGYING 正在播放；
- [ ] 防火墙允许 XINGYING/VRPN 访问专用网络。

### Ubuntu/ROS

- [ ] `ping <Windows-IP>` 或 `nc -zv <Windows-IP> 3883` 成功；
- [ ] 安装 `ros-noetic-vrpn-client-ros`；
- [ ] 启动 `roslaunch vrpn_client_ros sample.launch server:=<Windows-IP>`；
- [ ] 日志出现 `Found new sender`；
- [ ] `rostopic list` 能发现实际刚体名称；
- [ ] `rostopic echo -n 1 .../pose` 有位姿；
- [ ] `rostopic hz .../pose` 持续更新；
- [ ] 人工移动和旋转刚体时数据连续变化；
- [ ] 正式实验前录制短 bag 并用 `rosbag info` 检查。

## 11. 现场验证过的最短命令

已知现场参数：Windows `192.168.203.85`、刚体 `Tracker0`、默认端口 `3883`。

终端 1：

```bash
ping -c 2 192.168.203.85
nc -zv -w 2 192.168.203.85 3883

source /opt/ros/noetic/setup.bash
roslaunch vrpn_client_ros sample.launch server:=192.168.203.85
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash

rostopic list | grep '^/vrpn_client_node/' | sort
rostopic echo -n 1 /vrpn_client_node/Tracker0/pose
rostopic hz /vrpn_client_node/Tracker0/pose
```

项目用户再开终端 3：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

roslaunch nokov_mocap_monitor nokov_monitor.launch \
  start_vrpn:=false \
  server:=192.168.203.85 \
  tracker:=Tracker0
```

这组流程曾实际得到：

```text
/vrpn_client_node/Tracker0/pose
/vrpn_client_node/Tracker0/twist
/vrpn_client_node/Tracker0/accel
/mocap/scout_pose
/mocap/scout_odom
/mocap/status
```

原始位姿和桥接位姿均约 90 Hz，`/mocap/status` 为 `OK`。

---

参考资料：NOKOV《ROS 与 Nokov 动作捕捉系统的通信》、项目 `nokov_mocap_monitor` 使用说明，以及 2026-07-29 的现场成功接收记录。
