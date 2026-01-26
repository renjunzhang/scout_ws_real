# Git 使用指南（scout_ws）

面向 ROS 工作空间 `~/scout_ws` 的日常 Git 使用流程与注意事项。

## 1. 基础配置

首次在本仓库设置身份（已配置可跳过）：

```bash
git config user.name "<你的名字>"
git config user.email "<你的邮箱>"
```

查看/设置远端：

```bash
git remote -v
# 如需修改远端地址
git remote set-url origin https://github.com/renjunzhang/scout_ws_real
```

（当前默认分支为 `master`）

## 2. 常用工作流

在仓库根目录 `~/scout_ws` 下操作。

```bash
# 查看当前变更
git status

# 拉取远端更新并线性集成
git pull --rebase origin master

# 暂存更改（建议只添加源代码与文档）
# 典型 ROS 项目：主要提交 src/ 与 docs/
git add src docs .gitignore

# 创建提交
git commit -m "描述此次改动"

# 首次推送为分支设定上游
git push -u origin master
# 之后直接
git push
```

按块暂存（更细粒度）：

```bash
git add -p src/
```

## 3. 分支管理

```bash
# 新功能分支
git checkout -b feature/your-topic

# 推送并建立跟踪
git push -u origin feature/your-topic

# 在合并前与主线同步
git fetch origin
git rebase origin/master

# 回到主分支并合并（或在 GitHub 发起 PR）
git checkout master
git merge --no-ff feature/your-topic

# 合并后推送
git push
```

## 4. 变更审查

```bash
# 查看提交历史（简洁图形）
git log --oneline --graph --decorate --all

# 查看未暂存差异
git diff

# 查看已暂存差异
git diff --cached
```

## 5. 忽略与清理

- 根目录已有 `.gitignore`，已忽略 `build/`、`devel/`、`install/`、`*.bag` 等编译/大型产物。
- 包内 `src/ugv_sdk/.gitignore` 保留包特有的忽略规则，避免误提交临时文件。
- 若历史上已跟踪了不应提交的编译目录，可用：

```bash
git rm -r --cached build devel install
git commit -m "移除已跟踪的编译输出"
```

## 6. 远端与认证

使用 GitHub 推送时建议使用 Personal Access Token（PAT）。可选地启用凭据助手（注意安全）：

```bash
# 缓存一段时间（默认 15 分钟）
git config --global credential.helper cache
# 或持久存储到本地文件（慎用）
git config --global credential.helper store
```

## 7. 解决冲突（简要）

```bash
# 线性集成更新
git pull --rebase origin master
# 发生冲突时，编辑文件解决 >>> <<< 标记
# 标记解决后
git add <冲突文件>
# 继续 rebase
git rebase --continue
# 放弃 rebase
git rebase --abort
```

若使用合并（merge）产生冲突：

```bash
# 解决后
git add <冲突文件>
git merge --continue
```

## 8. 临时保存

```bash
# 暂存当前工作区（不提交）
git stash push -m "临时保存: 修复 X"
# 查看列表
git stash list
# 恢复并移除暂存
git stash pop
```

## 9. 回滚与恢复（慎用）

```bash
# 恢复某文件到 HEAD 状态（覆盖未提交改动）
# Git 2.23+ 可使用 restore
git restore --source=HEAD -- <path>
# 旧版方式
git checkout -- <path>

# 回滚某次提交（生成新提交）
git revert <commit-hash>

# 硬重置到某提交（会丢弃之后的提交）
# 非常危险，确认团队一致后再使用
git reset --hard <commit-hash>
```

## 10. 新环境克隆

```bash
# 在新机器或环境中
git clone https://github.com/renjunzhang/scout_ws_real ~/scout_ws
cd ~/scout_ws
# 查看忽略规则
git status
# 继续你的开发与构建（catkin/colcon）
```

---

最佳实践摘要：
- 只提交 `src/` 和文档，不提交 `build/`、`devel/` 等编译输出。
- 采用 `git pull --rebase` 保持线性历史，减少无意义的合并提交。
- 小步提交、清晰备注，分支开发，PR 合并。
- 发生冲突先阅读冲突标记，逐个文件解决再继续流程。
