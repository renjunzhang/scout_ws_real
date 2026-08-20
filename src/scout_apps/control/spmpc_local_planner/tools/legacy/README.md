# 历史工具

本目录保存已经退出活动运行链、但仍可能用于旧报告或旧 schema 复核的工具。
这些文件不安装到机器人 runtime，不得被实物运动 runner 调用，也不具有新 release
的放行权。

- `validate_realsense_timestamp_health.py`：历史 Python 时间戳门。活动实现是 C++
  `spmpc_realsense_timestamp_health_gate`；旧文件仅保留 schema v1 与历史数值语义参考。
