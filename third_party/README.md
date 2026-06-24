# third_party

本目录存放不直接纳入主 catkin workspace 的上游源码快照。

## LT_DWA

`LT_DWA/` 是 <https://github.com/flztiii/LT_DWA> 的 source-only vendor 副本，方便实物端 `git pull` 后拿到代码。它**不能直接放进 `src/` 编译**，因为上游包含 `local_planner`、`navigation` 等会与当前 workspace 冲突的 catkin package 名称。

当前状态：

- source-only vendor：是
- 主 catkin workspace 自动编译：否
- benchmark wrapper：已接入 `src/scout_apps/control/lt_dwa_official_wrapper/`，从本目录编译官方 core source，不直接把 vendor tree 放入 `src/`
- strict-fresh runnable baseline：通路可跑，formal 主表仍需 strict gate 证据

更多说明见：

- `LT_DWA/SCOUT_VENDOR_NOTES.md`
- `LT_DWA/SCOUT_ADAPTER_PLAN.md`
