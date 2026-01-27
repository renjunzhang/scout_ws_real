# nanoscan3_localization

## 功能说明

使用 **AMCL（Adaptive Monte Carlo Localization）** 算法在已知地图上进行机器人定位。


## 使用流程

### 第一步：建图（使用 Gmapping）

```bash
# 终端 1：启动底盘
roslaunch scout_bringup scout_mini_robot_base.launch

# 终端 2：启动激光雷达
roslaunch nanoscan3_bringup nanoscan3_front.launch

# 终端 3：启动 gmapping 建图
roslaunch nanoscan3_mapping scout_nanoscan3_gmapping.launch use_rviz:=true

# 终端 4：控制机器人移动建图
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 建图完成后保存地图（统一放到 scout_maps 包内）
rosrun map_server map_saver -f $(rospack find scout_maps)/maps/map122
```

### 第二步：定位（使用 AMCL）

```bash
# 终端 1：启动底盘
roslaunch scout_bringup scout_mini_robot_base.launch

# 终端 2：启动激光雷达
roslaunch nanoscan3_bringup nanoscan3_front.launch

# 终端 3：启动 AMCL 定位（可按需指定地图）
roslaunch nanoscan3_localization scout_nanoscan3_amcl.launch use_rviz:=true \
  map_file:=$(rospack find scout_maps)/maps/map122.yaml

# 在 RViz 中使用 "2D Pose Estimate" 设置初始位姿
# 之后机器人会自动跟踪定位
```

## 参数调优指南

### 关键参数说明

#### 粒子数量
```xml
<param name="min_particles" value="500" />   <!-- 最少粒子数 -->
<param name="max_particles" value="5000" />  <!-- 最多粒子数 -->
```
- **增大**：定位更准确，计算量增加
- **减小**：计算更快，定位可能不稳定

#### 更新阈值
```xml
<param name="update_min_d" value="0.2" />  <!-- 移动距离阈值(m) -->
<param name="update_min_a" value="0.5" />  <!-- 旋转角度阈值(rad) -->
```
- **增大**：更新频率降低，计算量减少，定位滞后
- **减小**：更新更频繁，计算量增加，定位更实时

#### 里程计噪声
```xml
<param name="odom_alpha1" value="0.2" />  <!-- 旋转→旋转噪声 -->
<param name="odom_alpha2" value="0.2" />  <!-- 平移→旋转噪声 -->
<param name="odom_alpha3" value="0.2" />  <!-- 平移→平移噪声 -->
<param name="odom_alpha4" value="0.2" />  <!-- 旋转→平移噪声 -->
```
- 底盘里程计**越准确**，这些值设置**越小**（0.1-0.2）
- 底盘里程计**越不准**，这些值设置**越大**（0.3-0.5）

### 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 定位抖动严重 | 粒子数太少 | 增大 `max_particles` 到 8000-10000 |
| 定位延迟大 | 更新阈值太高 | 减小 `update_min_d` 到 0.1 |
| 定位跑飞 | 里程计噪声参数不对 | 增大 `odom_alpha` 值 |
| 初始位姿不准 | 粒子分布不够广 | 增大 `initial_cov_xx/yy/aa` |
| CPU 占用高 | 粒子数太多 | 减小 `max_particles` 到 3000 |

## 文件结构

```
nanoscan3_localization/
├── launch/
│   └── scout_nanoscan3_amcl.launch    # AMCL 定位启动文件（默认读取 scout_maps/maps/map122.yaml）
├── config/
│   └── amcl.rviz                       # RViz 配置文件
└── README.md                            # 本说明文档
```

### 地图文件位置

地图文件统一存放在 `scout_maps` 包内的 `maps/` 目录，例如：`map122.yaml` / `map122.pgm`。

## 依赖安装

```bash
sudo apt-get install ros-noetic-amcl ros-noetic-map-server
```

## 参考资料

- [AMCL 官方文档](http://wiki.ros.org/amcl)
- [Navigation Stack 教程](http://wiki.ros.org/navigation/Tutorials)
- [AMCL 参数调优指南](http://wiki.ros.org/amcl#Parameters)

---

**创建日期**：2026年1月28日  
**维护者**：GitHub Copilot
