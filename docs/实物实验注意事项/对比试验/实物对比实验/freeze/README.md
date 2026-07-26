# S-MPCC 正式执行冻结产物

本目录对应现场协议 `SMPCC-REAL-40-88-v1.0`。

- `freeze_manifest.template.yaml` 只冻结 manifest 的必填结构，不代表正式采集已经准入。
- 必须先将模板和它引用的路径、配置、随机表、标定、分析工具与 smoke 报告全部提交，得到工作区干净的 Git HEAD；该 HEAD 是执行 manifest 中唯一允许填写的 `git_revision`。
- 只有具体配置、路径、C1/C2、随机表、P3/P4 决策证据、分析工具、K6/actual-zero/recorder smoke 报告全部归档，所有 gate 均为 `true` 后，才能由受控冻结流程生成并填写 `freeze_manifest.yaml`，把 `status` 改为 `GO` 并分配唯一 `FREEZE_ID`。`validate_spmpc_formal_freeze.py` 只读校验该结果，不生成、修改或签署 manifest。
- 禁止手工复制模板后仅修改 `status`/`freeze_id` 来绕过准入。正式启动必须调用 `validate_spmpc_formal_freeze.py`，逐项验证版本、运行参数、文件路径和 SHA-256，并在 manifest 外保存 validation report 及本次 manifest hash。
- `freeze_manifest.yaml`、`freeze_validation_report.txt` 和 `freeze_manifest_sha256.txt` 是由上述干净 HEAD 派生的本地执行态文件，仅对这三个精确文件名忽略。忽略只是为了不污染被冻结的 clean HEAD；三个文件仍必须只读复制到当批正式数据的外部归档目录。不得忽略 `paths/`、`configs/`、`randomization/`、`calibration/` 或任何被 manifest 引用的冻结资产。
- 当前仅存在模板，因此正式采集状态仍为 `NO-GO`。
- 第一条正式 trial 开始后，manifest 及其引用的全部产物只读。任何方法、路径、容器、分析规则或软件变化都需要新的 `FREEZE_ID` 和独立正式数据集。

预期子目录为 `paths/`、`configs/`、`randomization/` 和 `calibration/`。大体积 rosbag 留在 Git 仓库外；这里只归档不可变标识、hash 和报告。
