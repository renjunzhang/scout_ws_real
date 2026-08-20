# SPMPC 离线工具

本目录只承载非运行时工具，不被 CMake 安装到机器人部署空间：

- `analysis/`：bag/CSV 的只读分析、postflight、比较器准备和报告生成；
- `codegen/acados/`：CasADi 模型、代价、约束、参数 manifest 和 acados solver 生成入口。

工具不得被 launch 或控制主链调用。Python 回归测试统一位于 `../test/python/`，运行方式：

```bash
cd /home/a/scout_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
```
