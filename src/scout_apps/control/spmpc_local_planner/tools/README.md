# SPMPC 离线工具

本目录只承载非运行时工具，不被 CMake 安装到机器人部署空间：

- `analysis/`：bag/CSV 的只读分析、postflight、development artifact/freeze 草案准备和报告生成；
- `codegen/acados/`：CasADi 模型、代价、约束、参数 manifest 和 acados solver 生成入口。
- `legacy/`：仅用于旧证据复核、不得放行当前协议的历史工具。

工具不得被 launch 或控制主链调用。Python 回归测试统一位于 `../test/python/`，运行方式：

```bash
cd /home/a/scout_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
```

## Delay-augmented capsule 的唯一生成入口

完整生成 manifest、C/H/JSON 和共享库只使用下面这一条命令；不要手工调用生成目录中的 `Makefile`，也不要从其他工作树复制 `.so`：

```bash
cd /home/a/scout_ws
ACADOS_SOURCE_DIR=/absolute/path/to/acados \
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/control/spmpc_local_planner/tools/codegen/acados/generate_delay_augmented_phase_acados.py
```

`generated/acados/spmpc_delay_augmented_phase/` 被 git 忽略。fresh clone 只有已提交的 manifest，没有 capsule 的 C/H/JSON 或 `.so`；默认 CMake 配置因此构建可配置的 stub，所有实际依赖 capsule 的在线用例会 skip，安装产物也不会携带 capsule ELF 依赖。

生成完成后必须重新配置 CMake。本地开发验证需要显式 opt-in：

```bash
-DSPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE=ON
```

该选项为 `ON` 时，如果 `ACADOS_SOURCE_DIR` 或生成的 `.so` 缺失，配置会直接失败；编译真实 wrapper 时，安装规则会同时安装 capsule。安装后至少检查：

```bash
SPMPC_INSTALL_PREFIX=/absolute/path/to/install
test -f "$SPMPC_INSTALL_PREFIX/lib/libacados_ocp_solver_spmpc_delay_augmented_phase.so"
test ! -e "$SPMPC_INSTALL_PREFIX/include/spmpc_local_planner/simulation"
LD_LIBRARY_PATH="$SPMPC_INSTALL_PREFIX/lib:$ACADOS_SOURCE_DIR/lib:$ACADOS_SOURCE_DIR/lib64" \
  ldd "$SPMPC_INSTALL_PREFIX/lib/libspmpc_solver_delay_augmented_acados.so"
```

`ldd` 不得出现 `libacados_ocp_solver_spmpc_delay_augmented_phase.so => not found`。默认 stub 安装则反向检查该 capsule 不存在，且 `readelf -d` 中没有对应的 `NEEDED` 项。

当前 manifest 尚未用可信的 source/binary hash 绑定生成的 `.so`。因此显式 opt-in 也只允许本地开发验证，不构成 formal 或 robot release 证据；在该身份链闭合前保持 **NO-GO**。
