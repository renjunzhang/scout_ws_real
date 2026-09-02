# 唯一主线 Stage 3-D3：纯内存离散 Acados OCP

> 日期：2026-09-03
>
> 分支：`spmpc-mainline`
>
> 起始提交：`01b886d 文档：记录模块化符号图并推进唯一主线至D2d`
>
> 状态：`OCP_ASSEMBLED_AND_CONSISTENT / NO_ARTIFACT / DEV_UNVALIDATED`

## 1. 本阶段解决的问题

Stage 3-D2d 已有与独立数值 oracle 同构的 CasADi graph，但还没有把 graph 的每个
节点字段、bounds 和求解器数值选项连接到真实 Acados 0.5.4 OCP description。本阶段
只完成纯内存装配和反向校验：

- 冻结唯一的 backend-neutral development solver-option snapshot；
- 将 discrete dynamics、三类 cost 和三类 constraint 显式装配到 `AcadosOcp`；
- 调用 `make_consistent()` 后检查 Acados 归一化结果没有改变 typed authority；
- 组合记录 graph、bounds、solver options、symbolic expressions 和 backend 身份；
- 不实例化 `AcadosOcpSolver`，不 codegen，不生成 JSON/C/header/动态库。

因此 D3 不需要 bag 才能完成，也不授予 production 权限。七项 caller-supplied
constraint bounds 继续保持 `DEV_UNVALIDATED`，target latency 继续保持
`NOT_BENCHMARKED`。

## 2. 模块边界

OCP 路径没有留在单一大 adapter 中，而是按职责拆分：

```text
solver_options.py
  backend-neutral typed numerical authority

acados_ocp_contract.py
  dependency-free assembly metadata and composite identity

acados_backend.py
  lazy backend loading + backend/source/expression identity

acados_solver_options_adapter.py
  typed snapshot -> Acados setters + normalized-value validation

acados_ocp_validation.py
  dimensions + expressions + node types + bounds/x0/options validation

acados_ocp_adapter.py
  typed input checks + in-memory assembly orchestration only
```

顶层 import 不加载 CasADi、NumPy 或 `acados_template`。残缺或不兼容的 backend
统一报告 `AcadosOcpDependencyError`；装配后合同漂移统一报告
`AcadosOcpConstructionError`。既有测试 mock 边界继续停在 adapter 的 lazy loader，
没有为拆分引入兼容运行路径。

## 3. 唯一 solver-option snapshot

开发候选固定为：

```text
N=60, dt=1/30 s, Tf=2 s
time_steps = 60 个显式 1/30 s
cost_scaling = 61 个显式 1.0
integrator = DISCRETE
NLP = SQP_RTI, max_iter=1
Hessian = EXACT, exact dyn/cost/constraint enabled
regularization = PROJECT
globalization = FIXED_STEP
QP = PARTIAL_CONDENSING_HPIPM, cond_N=60, hpipm_mode=BALANCE
```

snapshot 自身不可变并具有 canonical semantic SHA256。`Fraction` 时间值只在 backend
adapter 边界显式转换成 Acados 需要的 float NumPy array；布尔 Hessian 标志显式转换
成后端整数表示。`make_consistent()` 后逐项反查已冻结字段、time steps 和 cost
scaling，避免 setter 或 backend default 静默改写候选。

这些数值是开发候选，不是性能结论。后续若 target profiling 改变 condensing、
Hessian 或其他已冻结选项，必须产生新的 snapshot 和 artifact identity。D3 尚未冻结
codegen/compiler options；它们属于 D4 manifest 的显式输入。

## 4. OCP 节点映射

装配后的固定维度为：

```text
N=60, NX=48, NU=3, NP=162
nh_0=5, nh=5, nh_e=1
nbx_0=48, nbu=3
```

节点覆盖明确为：

- dynamics：只使用 `model.disc_dyn_expr`，continuous dynamics 为空；
- initial/path cost：同一 stage total，类型均为 `EXTERNAL`；
- terminal cost：只使用 `x_N/p_N` 的 terminal total，类型为 `EXTERNAL`；
- stage 0/path `h`：`[q_issue_v,q_issue_omega,a_issue,alpha_issue,xi_k]`；
- terminal `h_e`：只有 `xi_N`，表达式中没有 `u_N`；
- control box：`[j_issue_v,j_issue_omega,v_s]` 对应 `[0,1,2]`；
- initial state：完整 48 维全零占位，runtime 必须替换全部 `lbx_0/ubx_0`；
- parameters：162 维全零占位，runtime 必须设置 `k=0..60` 全部 61 行。

校验同时要求未声明的 path/terminal state box、general constraint、phi constraint 和
所有 slack 维度为零。14 项 lower/upper diagnostic residual 不连接到 Acados `h`，
B0/Bslosh 都没有液体 hard constraint。

## 5. 组合身份

纯内存 assembly identity 组合绑定：

- capacity raw bytes、development layout、solver parameter layout；
- graph semantic hash、七项 bounds snapshot、solver-option snapshot；
- dynamics、stage/terminal cost、stage/terminal `h` 的 CasADi serialization hash；
- CasADi version、Acados git commit；
- 选定 `acados_template` Python interface 源文件 hash；
- Python interface source root 与 Acados library source root 的绑定状态；
- 全部 node mapping、constraint/control order 和 bounds。

这仍不是 artifact hash。`MATCHED_SOURCE_ROOT` 只证明本次内存装配的 Python interface
与 library metadata 指向同一源码根；若为 `UNRESOLVED_SOURCE_ROOT_MISMATCH`，D3
只记录事实，D4 生成/发布边界必须拒绝或先消除该状态。

## 6. 无副作用与失败边界

成功和失败路径都在隔离临时目录验证：

- 不创建 code export directory；
- 不生成 JSON、C、header、共享库或 manifest；
- typed graph/options 错误先于 backend 加载失败；
- 缺失或残缺 backend 产生明确 dependency failure；
- 维度、表达式、cost/constraint type、x0、bounds 或 solver option 漂移 fail closed；
- B0/Bslosh 的 OCP identity 明确声明未来必须共用一个 artifact，运行时只允许两个
  liquid coefficient 不同。

## 7. 分阶段提交

```text
374c280 主线生成：冻结离散OCP开发求解选项合同
5b6506f 主线生成：装配纯内存离散Acados OCP合同
5311c64 主线重构：拆分Acados后端绑定、选项映射与OCP校验
```

## 8. 当前验证

- 默认环境 mainline Python discovery：159/159，通过；真实 backend 项 7 项按设计跳过；
- 显式 CasADi 3.7.2 + Acados 0.5.4：159/159，无跳过；
- Acados OCP 专项：13/13；
- Stage 2 execution golden：10/10；
- Stage 2 history/prefix/projection golden：14/14；
- Ruff check/format、Python 3.8 grammar、`git diff --check`：通过。

CMake 已登记 solver-options 与 Acados OCP 两个专项测试。显式 backend CI 必须设置
`SPMPC_REQUIRE_ACADOS_BACKEND=1`，避免缺少 `acados_template` 时把集成测试跳过误读为
真实后端已验证。

## 9. 下一阶段

Stage 3-D4 单独实现可审计的 `DEV_UNVALIDATED` artifact：

1. 从 D3 assembly 调用真实 Acados codegen，生成目录必须显式、隔离且可枚举；
2. 从同一 typed authority 生成唯一 `model_contract.json` 和 C++ header，不手写第二套
   names/offsets/dimensions；
3. manifest 冻结 codegen/compiler options、backend/source identity、生成命令和输入
   hashes，并记录实际生成文件的可复算 SHA256；
4. header 以 compile-time assertions 固定 `NX=48/NU=3/NP=162/N=60`；
5. 篡改、缺文件、身份不一致和 source-root mismatch 必须 fail closed；
6. 证明 B0/Bslosh 使用同一 graph、OCP、options 和 artifact hash；
7. runtime `delay/m/beta/selector/runtime_schedule_hash` 不进入 artifact identity；
8. 不读取 bag 或 Stage 1 production gate，不在 D4 伪造 production manifest。

wrapper、objective diagnostics、warm start 和 ROS orchestration 继续放在后续独立提交，
避免 codegen、runtime 和发布路径重新耦合。
