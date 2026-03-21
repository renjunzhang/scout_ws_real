# 第三方依赖记录

本文记录 `scout_ws` 中以源码形式并入主仓库的第三方依赖，目标是避免后续在 A 电脑开发、B 电脑部署时搞不清来源、导入方式和维护约定。

## 1. `realsense-ros`

### 基本信息

- 名称：`realsense-ros`
- 上游仓库：`https://github.com/IntelRealSense/realsense-ros`
- 本地路径：`/home/a/scout_ws/src/third_party/realsense-ros`
- 导入日期：`2026-03-21`

### 当前导入方式

当前采用的是：

- **vendor 进主仓库**

也就是说：

- 不作为 `submodule`
- 不保留独立 `.git`
- 不要求 B 电脑额外初始化子模块
- A 电脑提交后，B 电脑直接 `git pull` 即可获得完整源码

这次实际使用的导入命令为：

```bash
mkdir -p /home/a/scout_ws/src/third_party
rsync -a --exclude='.git' /home/a/RealSense_ws/src/realsense-ros/ /home/a/scout_ws/src/third_party/realsense-ros/
```

### 为什么当前采用 vendor 方案

当前项目更适合把 `realsense-ros` 直接并入主仓库，原因是：

- 当前目标是尽快把 RealSense 视觉测量链在 A/B 两台机器上统一起来
- `submodule` 会增加初始化、更新和部署复杂度
- 工控机 B 电脑更适合直接 `git pull` 后即可编译
- 你的自定义逻辑会另外放在独立包里，例如后续的 `realsense_liquid_measurement`

### 当前维护约定

当前建议遵守下面这些规则：

- 尽量**不要直接修改** `src/third_party/realsense-ros` 源码
- 如果必须修改，要在本文件补充“本地修改记录”
- 你自己的业务逻辑尽量放在独立包里，不要和第三方源码混写
- 如果将来需要同步上游更新，优先重新 vendor 一次，再处理差异

### 本地修改记录

当前状态：

- `none`

如果后续修改了第三方源码，建议按下面格式补充：

```text
日期：
修改文件：
修改原因：
是否影响 B 电脑部署：
是否还能方便同步上游：
```

### A / B 电脑协作约定

当前工作流建议固定为：

1. 在 A 电脑完成第三方源码导入或升级
2. 在 `scout_ws` 主仓库中直接提交相关路径
3. 推送到远端主仓库
4. 在 B 电脑执行 `git pull`
5. 在 B 电脑重新编译工作空间

因此对当前项目，`realsense-ros` 的 Git 管理结论是：

- **作为 `scout_ws` 主仓库里的 `third_party` 源码直接管理**
- **不单独拆成子仓库工作流**

## 2. 一句话原则

如果某个第三方依赖已经明确要跟随 `scout_ws` 一起在 A/B 两台机器上同步，并且当前更关心部署稳定性而不是独立升级灵活性，那么优先考虑：

- **vendor 到 `src/third_party/`**

而不是：

- `submodule`
- 单独维护一个平行工作空间
