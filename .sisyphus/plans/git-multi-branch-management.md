# Git 多分支管理 — 通用驱动 + 平台实现

## TL;DR

> **Quick Summary**: 重构仓库为"通用驱动在 master + 平台实现在分支"的架构，支持 STM32F407、MSPM0G3507、CH32、AT32 四个平台，配套自动同步脚本和工作流文档。
> 
> **Deliverables**:
> - 清理后的 master 分支（仅含通用驱动 + 滤波代码）
> - 4 个平台分支（stm32f407, mspm0g3507, ch32, at32）
> - 自动同步脚本（将 master 更新同步到各平台分支）
> - 分支管理工作流文档
> 
> **Estimated Effort**: Medium
> **Parallel Execution**: YES — 3 waves
> **Critical Path**: Task 1 → Task 2 → Task 3/4/5/6 → Task 7

---

## Context

### Original Request
用户希望一个仓库实现：master 分支存放陀螺仪通用驱动与滤波算法，多个分支分别实现不同 MCU 的陀螺仪应用。

### Interview Summary
**Key Discussions**:
- **目标平台**: STM32F407（已有）、MSPM0G3507、CH32、AT32
- **Keil MDK 项目文件**: 每个平台分支独立保留自己的 .uvprojx
- **CI/CD**: 不需要
- **分支保护**: 不需要（个人项目）
- **通用层同步**: 自动同步脚本

**Research Findings**:
- 当前所有分支指向同一提交 `0bf1821`，实质上是同一份代码的多个副本
- 三层架构已存在：驱动层（lsm6dsr.c/h）、业务层（bsp_lsm6dsr.c/h）、测试层（test_lsm6dsr.c/h）
- 驱动层已通过 `lsm6dsr_io_t` 回调结构体实现 I/O 抽象，天然支持平台隔离

---

## Work Objectives

### Core Objective
将仓库重构为"通用层在 master + 平台实现在长期分支"的架构，实现跨 MCU 平台的代码复用与独立开发。

### Concrete Deliverables
- master 分支：仅含 `lsm6dsr.c/h` + `bsp_lsm6dsr.c/h` + 文档 + 同步脚本
- stm32f407 分支：基于 master，添加 STM32 HAL 桥接 + Keil 项目 + 测试
- mspm0g3507 分支：基于 master，创建 MSPM0G3507 平台桩文件
- ch32 分支：基于 master，创建 CH32 平台桩文件
- at32 分支：基于 master，创建 AT32 平台桩文件
- `scripts/sync-to-platforms.sh`：自动同步脚本
- `docs/BRANCHING.md`：分支管理工作流文档

### Definition of Done
- [ ] master 分支不含任何平台特定代码（无 test_lsm6dsr.c/h、无 .uvprojx、无 HAL 驱动）
- [ ] 每个平台分支可独立编译（或有明确的桩文件占位）
- [ ] 同步脚本能将 master 更新 merge 到所有平台分支
- [ ] 文档描述了完整的分支工作流

### Must Have
- master 分支只包含平台无关的通用代码
- 每个平台分支基于 master，通过 merge 保持同步
- 自动同步脚本可一键更新所有平台分支
- 提交规范遵循 Conventional Commits

### Must NOT Have (Guardrails)
- 不在 master 上引入任何平台特定代码
- 不使用 git submodule（过于复杂）
- 不设置 CI/CD（用户明确不需要）
- 不改变现有代码的功能逻辑（只做文件重组）
- 不删除现有的 docs/ 目录内容

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed.

### Test Decision
- **Infrastructure exists**: N/A（Git 管理任务，非代码实现）
- **Automated tests**: None
- **Framework**: N/A

### QA Policy
每个任务必须包含 agent-executed QA 场景。
Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **Git 操作**: Use Bash — 执行 git 命令，验证分支状态、文件内容、提交历史
- **脚本测试**: Use Bash — 运行同步脚本，验证分支更新结果

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — 基础准备):
├── Task 1: 审计当前文件，分类通用 vs 平台特定 [quick]
└── Task 2: 清理 master 分支，仅保留通用代码 [unspecified-high]

Wave 2 (After Wave 1 — 平台分支创建，MAX PARALLEL):
├── Task 3: 创建 stm32f407 平台分支 [unspecified-high]
├── Task 4: 创建 mspm0g3507 平台分支 [unspecified-high]
├── Task 5: 创建 ch32 平台分支 [unspecified-high]
├── Task 6: 创建 at32 平台分支 [unspecified-high]
└── Task 7: 创建自动同步脚本 [quick]

Wave 3 (After Wave 2 — 文档与收尾):
├── Task 8: 创建分支管理工作流文档 [writing]
└── Task 9: 清理旧分支 + 推送到远程 [quick]

Wave FINAL (After ALL tasks):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Real manual QA (unspecified-high)
└── Task F4: Scope fidelity check (deep)
-> Present results -> Get explicit user okay

Critical Path: Task 1 → Task 2 → Task 3 → Task 7 → Task 8 → F1-F4 → user okay
Parallel Speedup: ~60% faster than sequential
Max Concurrent: 5 (Wave 2)
```

### Dependency Matrix

| Task | Depends On | Blocks |
|------|-----------|--------|
| 1 | - | 2 |
| 2 | 1 | 3, 4, 5, 6 |
| 3 | 2 | 7, 8 |
| 4 | 2 | 7, 8 |
| 5 | 2 | 7, 8 |
| 6 | 2 | 7, 8 |
| 7 | 3, 4, 5, 6 | 9 |
| 8 | 3, 4, 5, 6 | 9 |
| 9 | 7, 8 | F1-F4 |

### Agent Dispatch Summary

- **Wave 1**: 2 tasks — T1 → `quick`, T2 → `unspecified-high`
- **Wave 2**: 5 tasks — T3-T6 → `unspecified-high`, T7 → `quick`
- **Wave 3**: 2 tasks — T8 → `writing`, T9 → `quick`
- **FINAL**: 4 tasks — F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`, F4 → `deep`

---

## TODOs

- [x] 1. 审计当前文件，分类通用 vs 平台特定

  **What to do**:
  - 列出仓库中所有源文件（.c/.h）
  - 分类为：通用（平台无关）vs 平台特定
  - 通用文件：`lsm6dsr.c/h`（驱动层）、`bsp_lsm6dsr.c/h`（业务层/滤波）
  - 平台特定文件：`test_lsm6dsr.c/h`（HAL 桥接）、MDK-ARM/（Keil 项目）、Drivers/（HAL 驱动）、Core/Src/main.c 等
  - 生成分类报告文件 `.sisyphus/evidence/task-1-file-audit.md`

  **Must NOT do**:
  - 不修改任何文件内容
  - 不删除任何文件

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 只需读取文件列表和内容进行分类，不涉及修改
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 1 (sequential — Task 2 depends on this)
  - **Blocks**: Task 2
  - **Blocked By**: None (can start immediately)

  **References**:
  - `Core/Src/lsm6dsr.c` — 平台无关驱动层实现
  - `Core/Inc/lsm6dsr.h` — 驱动层头文件，包含 `lsm6dsr_io_t` 回调结构体定义
  - `Core/Src/bsp_lsm6dsr.c` — 业务层（滤波、偏置跟踪）
  - `Core/Inc/bsp_lsm6dsr.h` — 业务层头文件
  - `Core/Src/test_lsm6dsr.c` — 测试层（STM32 HAL 桥接 + P1~P19 测试）
  - `Core/Inc/test_lsm6dsr.h` — 测试层头文件
  - `MDK-ARM/` — Keil MDK 项目文件目录
  - `Drivers/` — STM32 HAL 驱动目录

  **Acceptance Criteria**:
  - [ ] `.sisyphus/evidence/task-1-file-audit.md` 存在且包含完整分类
  - [ ] 通用文件列表包含 lsm6dsr.c/h 和 bsp_lsm6dsr.c/h
  - [ ] 平台特定文件列表包含 test_lsm6dsr.c/h 和 MDK-ARM/

  **QA Scenarios**:
  ```
  Scenario: 文件分类报告完整性
    Tool: Bash
    Preconditions: 仓库工作区干净
    Steps:
      1. cat .sisyphus/evidence/task-1-file-audit.md
      2. 检查报告中包含 "通用文件" 和 "平台特定文件" 两个分类
      3. 检查 lsm6dsr.c 和 bsp_lsm6dsr.c 出现在通用文件列表中
      4. 检查 test_lsm6dsr.c 出现在平台特定文件列表中
    Expected Result: 分类报告存在且分类正确
    Failure Indicators: 报告不存在或分类错误
    Evidence: .sisyphus/evidence/task-1-file-audit.md
  ```

  **Commit**: NO

- [x] 2. 清理 master 分支，仅保留通用代码

  **What to do**:
  - 切换到 master 分支
  - 删除所有平台特定文件：test_lsm6dsr.c/h、MDK-ARM/、Drivers/、Core/Src/main.c 等
  - 保留通用文件：lsm6dsr.c/h、bsp_lsm6dsr.c/h
  - 更新 .gitignore（确保排除构建产物）
  - 提交更改：`refactor: clean master to common-only driver and filter`

  **Must NOT do**:
  - 不删除通用文件（lsm6dsr.c/h、bsp_lsm6dsr.c/h）
  - 不修改通用文件的功能逻辑
  - 不删除 docs/ 目录
  - 不删除 .gitignore

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: 涉及 git 操作和文件重组，需要谨慎处理
  - **Skills**: [`git-master`]
    - `git-master`: 分支操作、提交管理

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 1 (sequential after Task 1)
  - **Blocks**: Task 3, 4, 5, 6
  - **Blocked By**: Task 1

  **References**:
  - `.gitignore` — 现有忽略规则，确保构建产物被排除
  - Task 1 的分类报告 — 确定哪些文件是平台特定的

  **Acceptance Criteria**:
  - [ ] `git ls-tree -r master --name-only | grep test_lsm6dsr` 无输出
  - [ ] `git ls-tree -r master --name-only | grep .uvprojx` 无输出
  - [ ] `git ls-tree -r master --name-only | grep lsm6dsr.c` 有输出
  - [ ] `git ls-tree -r master --name-only | grep bsp_lsm6dsr.c` 有输出

  **QA Scenarios**:
  ```
  Scenario: master 分支只含通用代码
    Tool: Bash
    Preconditions: Task 2 已完成
    Steps:
      1. git checkout master
      2. git ls-tree -r HEAD --name-only | grep -E "(test_lsm6dsr|\.uvprojx)"
      3. git ls-tree -r HEAD --name-only | grep -E "(lsm6dsr\.c|bsp_lsm6dsr\.c)"
      4. 检查步骤 2 无输出，步骤 3 有输出
    Expected Result: master 分支不含平台特定文件，包含通用文件
    Failure Indicators: 步骤 2 有输出或步骤 3 无输出
    Evidence: .sisyphus/evidence/task-2-master-clean.txt

  Scenario: 通用文件内容未被修改
    Tool: Bash
    Preconditions: Task 2 已完成
    Steps:
      1. git diff HEAD~1 HEAD -- Core/Src/lsm6dsr.c Core/Inc/lsm6dsr.h Core/Src/bsp_lsm6dsr.c Core/Inc/bsp_lsm6dsr.h
      2. 检查 diff 为空或只有格式变化
    Expected Result: 通用文件内容未改变
    Failure Indicators: diff 显示功能逻辑变化
    Evidence: .sisyphus/evidence/task-2-common-files-unchanged.txt
  ```

  **Commit**: YES
  - Message: `refactor: clean master to common-only driver and filter`
  - Pre-commit: `git status`

- [x] 3. 创建 stm32f407 平台分支

  **What to do**:
  - 从 master 创建 stm32f407 分支：`git checkout -b stm32f407 master`
  - 添加 STM32F407 平台特定文件：
    - `Core/Src/test_lsm6dsr.c` — STM32 HAL I2C 桥接 + P1~P19 测试
    - `Core/Inc/test_lsm6dsr.h` — 测试层头文件
    - `MDK-ARM/` — Keil MDK 项目文件
    - `Drivers/` — STM32 HAL 驱动
    - `Core/Src/main.c` — 主程序入口
    - `Core/Src/stm32f4xx_it.c` — 中断处理
    - `Core/Src/system_stm32f4xx.c` — 系统初始化
    - `Core/Inc/stm32f4xx_hal_conf.h` — HAL 配置
    - `startup_stm32f407xx.s` — 启动文件
  - 提交更改：`feat(stm32f407): add STM32F407 platform-specific files`

  **Must NOT do**:
  - 不修改通用文件（lsm6dsr.c/h、bsp_lsm6dsr.c/h）
  - 不删除 docs/ 目录

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: 涉及 git 分支操作和文件重组
  - **Skills**: [`git-master`]
    - `git-master`: 分支创建、文件添加、提交管理

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 4, 5, 6)
  - **Blocks**: Task 7, 8
  - **Blocked By**: Task 2

  **References**:
  - `Core/Src/test_lsm6dsr.c` — 现有 STM32 HAL 桥接实现，包含 `lsm6dsr_io` I/O 实例定义
  - `Core/Inc/test_lsm6dsr.h` — 测试层头文件，定义 P1~P19 测试编号
  - `MDK-ARM/` — Keil MDK 项目文件，包含 .uvprojx 和 .uvoptx
  - `Drivers/STM32F4xx_HAL_Driver/` — STM32 HAL 驱动库
  - `Drivers/CMSIS/` — ARM CMSIS 头文件

  **Acceptance Criteria**:
  - [ ] `git branch -a | grep stm32f407` 有输出
  - [ ] `git ls-tree -r stm32f407 --name-only | grep test_lsm6dsr.c` 有输出
  - [ ] `git ls-tree -r stm32f407 --name-only | grep .uvprojx` 有输出

  **QA Scenarios**:
  ```
  Scenario: stm32f407 分支包含平台特定文件
    Tool: Bash
    Preconditions: Task 3 已完成
    Steps:
      1. git checkout stm32f407
      2. git ls-tree -r HEAD --name-only | grep test_lsm6dsr.c
      3. git ls-tree -r HEAD --name-only | grep .uvprojx
      4. git ls-tree -r HEAD --name-only | grep lsm6dsr.c
      5. 检查步骤 2、3、4 都有输出
    Expected Result: stm32f407 分支包含平台特定文件和通用文件
    Failure Indicators: 任何步骤无输出
    Evidence: .sisyphus/evidence/task-3-stm32f407-branch.txt

  Scenario: stm32f407 分支基于 master
    Tool: Bash
    Preconditions: Task 3 已完成
    Steps:
      1. git merge-base --is-ancestor master stm32f407
      2. 检查退出码为 0
    Expected Result: stm32f407 分支包含 master 的所有提交
    Failure Indicators: 退出码非 0
    Evidence: .sisyphus/evidence/task-3-stm32f407-ancestor.txt
  ```

  **Commit**: YES
  - Message: `feat(stm32f407): add STM32F407 platform-specific files`
  - Pre-commit: `git status`

- [x] 4. 创建 mspm0g3507 平台分支

  **What to do**:
  - 从 master 创建 mspm0g3507 分支：`git checkout -b mspm0g3507 master`
  - 创建 MSPM0G3507 平台桩文件：
    - `ports/mspm0g3507/test_lsm6dsr.c` — MSPM0G3507 I2C 桥接桩（TODO 注释）
    - `ports/mspm0g3507/test_lsm6dsr.h` — 测试层头文件桩
    - `ports/mspm0g3507/README.md` — 平台实现说明
  - 提交更改：`feat(mspm0g3507): create MSPM0G3507 platform stub`

  **Must NOT do**:
  - 不修改通用文件
  - 不复制 STM32 代码（桩文件应为空壳）

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: 涉及 git 分支操作和文件创建
  - **Skills**: [`git-master`]
    - `git-master`: 分支创建、文件添加、提交管理

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 3, 5, 6)
  - **Blocks**: Task 7, 8
  - **Blocked By**: Task 2

  **References**:
  - `Core/Src/test_lsm6dsr.c` — 参考 STM32 实现的接口，创建桩文件时保持相同的函数签名
  - `Core/Inc/test_lsm6dsr.h` — 参考头文件结构
  - `lsm6dsr_io_t` 结构体定义 — 桩文件需要实现 read/write 回调

  **Acceptance Criteria**:
  - [ ] `git branch -a | grep mspm0g3507` 有输出
  - [ ] `git ls-tree -r mspm0g3507 --name-only | grep ports/mspm0g3507/` 有输出
  - [ ] `git ls-tree -r mspm0g3507 --name-only | grep lsm6dsr.c` 有输出

  **QA Scenarios**:
  ```
  Scenario: mspm0g3507 分支包含桩文件
    Tool: Bash
    Preconditions: Task 4 已完成
    Steps:
      1. git checkout mspm0g3507
      2. git ls-tree -r HEAD --name-only | grep ports/mspm0g3507/
      3. cat ports/mspm0g3507/test_lsm6dsr.c | grep -i "TODO"
      4. 检查步骤 2 有输出，步骤 3 有 TODO 注释
    Expected Result: mspm0g3507 分支包含桩文件且有 TODO 标记
    Failure Indicators: 桩文件不存在或无 TODO 标记
    Evidence: .sisyphus/evidence/task-4-mspm0g3507-stub.txt
  ```

  **Commit**: YES
  - Message: `feat(mspm0g3507): create MSPM0G3507 platform stub`
  - Pre-commit: `git status`

- [x] 5. 创建 ch32 平台分支

  **What to do**:
  - 从 master 创建 ch32 分支：`git checkout -b ch32 master`
  - 创建 CH32 平台桩文件：
    - `ports/ch32/test_lsm6dsr.c` — CH32 I2C 桥接桩（TODO 注释）
    - `ports/ch32/test_lsm6dsr.h` — 测试层头文件桩
    - `ports/ch32/README.md` — 平台实现说明
  - 提交更改：`feat(ch32): create CH32 platform stub`

  **Must NOT do**:
  - 不修改通用文件
  - 不复制 STM32 代码

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: 涉及 git 分支操作和文件创建
  - **Skills**: [`git-master`]
    - `git-master`: 分支创建、文件添加、提交管理

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 3, 4, 6)
  - **Blocks**: Task 7, 8
  - **Blocked By**: Task 2

  **References**:
  - `Core/Src/test_lsm6dsr.c` — 参考 STM32 实现的接口
  - `Core/Inc/test_lsm6dsr.h` — 参考头文件结构
  - `lsm6dsr_io_t` 结构体定义 — 桩文件需要实现 read/write 回调

  **Acceptance Criteria**:
  - [ ] `git branch -a | grep ch32` 有输出
  - [ ] `git ls-tree -r ch32 --name-only | grep ports/ch32/` 有输出

  **QA Scenarios**:
  ```
  Scenario: ch32 分支包含桩文件
    Tool: Bash
    Preconditions: Task 5 已完成
    Steps:
      1. git checkout ch32
      2. git ls-tree -r HEAD --name-only | grep ports/ch32/
      3. cat ports/ch32/test_lsm6dsr.c | grep -i "TODO"
      4. 检查步骤 2 有输出，步骤 3 有 TODO 注释
    Expected Result: ch32 分支包含桩文件且有 TODO 标记
    Failure Indicators: 桩文件不存在或无 TODO 标记
    Evidence: .sisyphus/evidence/task-5-ch32-stub.txt
  ```

  **Commit**: YES
  - Message: `feat(ch32): create CH32 platform stub`
  - Pre-commit: `git status`

- [x] 6. 创建 at32 平台分支

  **What to do**:
  - 从 master 创建 at32 分支：`git checkout -b at32 master`
  - 创建 AT32 平台桩文件：
    - `ports/at32/test_lsm6dsr.c` — AT32 I2C 桥接桩（TODO 注释）
    - `ports/at32/test_lsm6dsr.h` — 测试层头文件桩
    - `ports/at32/README.md` — 平台实现说明
  - 提交更改：`feat(at32): create AT32 platform stub`

  **Must NOT do**:
  - 不修改通用文件
  - 不复制 STM32 代码

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: 涉及 git 分支操作和文件创建
  - **Skills**: [`git-master`]
    - `git-master`: 分支创建、文件添加、提交管理

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 3, 4, 5)
  - **Blocks**: Task 7, 8
  - **Blocked By**: Task 2

  **References**:
  - `Core/Src/test_lsm6dsr.c` — 参考 STM32 实现的接口
  - `Core/Inc/test_lsm6dsr.h` — 参考头文件结构
  - `lsm6dsr_io_t` 结构体定义 — 桩文件需要实现 read/write 回调

  **Acceptance Criteria**:
  - [ ] `git branch -a | grep at32` 有输出
  - [ ] `git ls-tree -r at32 --name-only | grep ports/at32/` 有输出

  **QA Scenarios**:
  ```
  Scenario: at32 分支包含桩文件
    Tool: Bash
    Preconditions: Task 6 已完成
    Steps:
      1. git checkout at32
      2. git ls-tree -r HEAD --name-only | grep ports/at32/
      3. cat ports/at32/test_lsm6dsr.c | grep -i "TODO"
      4. 检查步骤 2 有输出，步骤 3 有 TODO 注释
    Expected Result: at32 分支包含桩文件且有 TODO 标记
    Failure Indicators: 桩文件不存在或无 TODO 标记
    Evidence: .sisyphus/evidence/task-6-at32-stub.txt
  ```

  **Commit**: YES
  - Message: `feat(at32): create AT32 platform stub`
  - Pre-commit: `git status`

- [x] 7. 创建自动同步脚本

  **What to do**:
  - 在 master 分支创建 `scripts/sync-to-platforms.sh` 脚本
  - 脚本功能：
    - 遍历所有平台分支（stm32f407, mspm0g3507, ch32, at32）
    - 对每个分支执行 `git merge master`
    - 如果有冲突，暂停并提示用户手动解决
    - 合并成功后自动提交
  - 在 master 分支创建 `scripts/README.md` 说明脚本用法
  - 提交更改：`feat(scripts): add auto-sync script for platform branches`

  **Must NOT do**:
  - 不修改通用文件
  - 不自动解决冲突（冲突时暂停）

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 只需创建一个 shell 脚本
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 2 (after Tasks 3-6 complete)
  - **Blocks**: Task 9
  - **Blocked By**: Task 3, 4, 5, 6

  **References**:
  - `git merge` 文档 — 了解合并策略
  - 现有 .gitignore — 确保脚本不被忽略

  **Acceptance Criteria**:
  - [ ] `git ls-tree -r master --name-only | grep sync-to-platforms.sh` 有输出
  - [ ] 脚本包含遍历平台分支的逻辑
  - [ ] 脚本包含冲突检测逻辑

  **QA Scenarios**:
  ```
  Scenario: 同步脚本存在且可执行
    Tool: Bash
    Preconditions: Task 7 已完成
    Steps:
      1. git checkout master
      2. test -f scripts/sync-to-platforms.sh
      3. grep -q "stm32f407" scripts/sync-to-platforms.sh
      4. grep -q "mspm0g3507" scripts/sync-to-platforms.sh
      5. 检查所有步骤成功
    Expected Result: 同步脚本存在且包含所有平台分支
    Failure Indicators: 脚本不存在或缺少平台分支
    Evidence: .sisyphus/evidence/task-7-sync-script.txt

  Scenario: 同步脚本语法正确
    Tool: Bash
    Preconditions: Task 7 已完成
    Steps:
      1. bash -n scripts/sync-to-platforms.sh
      2. 检查退出码为 0
    Expected Result: 脚本语法正确
    Failure Indicators: 退出码非 0
    Evidence: .sisyphus/evidence/task-7-sync-script-syntax.txt
  ```

  **Commit**: YES
  - Message: `feat(scripts): add auto-sync script for platform branches`
  - Pre-commit: `bash -n scripts/sync-to-platforms.sh`

- [x] 8. 创建分支管理工作流文档

  **What to do**:
  - 在 master 分支创建 `docs/BRANCHING.md` 文档
  - 文档内容：
    - 分支策略说明（master 通用层 + 平台分支）
    - 新平台开发流程（从 master 创建分支、添加桩文件）
    - 通用层更新流程（修改 master、运行同步脚本）
    - 冲突解决指南
    - 提交规范说明
  - 提交更改：`docs: add branching workflow documentation`

  **Must NOT do**:
  - 不修改现有文档
  - 不添加不相关的文档

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: 纯文档编写任务
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 3 (with Task 9, after Tasks 3-6)
  - **Blocks**: Task 9
  - **Blocked By**: Task 3, 4, 5, 6

  **References**:
  - `docs/` — 现有文档目录，了解文档风格
  - Conventional Commits 规范 — 文档中需要引用

  **Acceptance Criteria**:
  - [ ] `git ls-tree -r master --name-only | grep BRANCHING.md` 有输出
  - [ ] 文档包含分支策略说明
  - [ ] 文档包含新平台开发流程
  - [ ] 文档包含通用层更新流程

  **QA Scenarios**:
  ```
  Scenario: 分支管理文档存在且完整
    Tool: Bash
    Preconditions: Task 8 已完成
    Steps:
      1. git checkout master
      2. test -f docs/BRANCHING.md
      3. grep -q "master" docs/BRANCHING.md
      4. grep -q "stm32f407" docs/BRANCHING.md
      5. grep -q "sync" docs/BRANCHING.md
      6. 检查所有步骤成功
    Expected Result: 文档存在且包含关键内容
    Failure Indicators: 文档不存在或缺少关键内容
    Evidence: .sisyphus/evidence/task-8-branching-doc.txt
  ```

  **Commit**: YES
  - Message: `docs: add branching workflow documentation`
  - Pre-commit: `git status`

- [x] 9. 清理旧分支 + 推送到远程

  **What to do**:
  - 删除不再需要的本地分支：
    - `demo/stm32f407`（功能已合并到 stm32f407）
    - `agents/refactor-lsm6dsr-common-driver`（功能已完成）
  - 推送所有分支到远程：
    - `git push origin master`
    - `git push origin stm32f407`
    - `git push origin mspm0g3507`
    - `git push origin ch32`
    - `git push origin at32`
  - 推送分支删除：
    - `git push origin --delete demo/stm32f407`
    - `git push origin --delete agents/refactor-lsm6dsr-common-driver`

  **Must NOT do**:
  - 不删除 master、stm32f407、mspm0g3507、ch32、at32 分支
  - 不强制推送（--force）

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 简单的 git 操作
  - **Skills**: [`git-master`]
    - `git-master`: 分支删除、远程推送

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 3 (after Tasks 7, 8)
  - **Blocks**: F1-F4
  - **Blocked By**: Task 7, 8

  **References**:
  - 远程仓库地址：https://github.com/Nanami7-7/LSM6DSR.git
  - 现有本地分支列表

  **Acceptance Criteria**:
  - [ ] `git branch -a | grep demo/stm32f407` 无输出
  - [ ] `git branch -a | grep agents/refactor-lsm6dsr-common-driver` 无输出
  - [ ] `git branch -a | grep stm32f407` 有输出
  - [ ] `git branch -a | grep mspm0g3507` 有输出
  - [ ] `git branch -a | grep ch32` 有输出
  - [ ] `git branch -a | grep at32` 有输出

  **QA Scenarios**:
  ```
  Scenario: 旧分支已删除
    Tool: Bash
    Preconditions: Task 9 已完成
    Steps:
      1. git branch -a | grep demo/stm32f407
      2. git branch -a | grep agents/refactor-lsm6dsr-common-driver
      3. 检查步骤 1 和 2 都无输出
    Expected Result: 旧分支已删除
    Failure Indicators: 任何步骤有输出
    Evidence: .sisyphus/evidence/task-9-old-branches-deleted.txt

  Scenario: 新分支已推送到远程
    Tool: Bash
    Preconditions: Task 9 已完成
    Steps:
      1. git fetch origin
      2. git branch -r | grep origin/stm32f407
      3. git branch -r | grep origin/mspm0g3507
      4. git branch -r | grep origin/ch32
      5. git branch -r | grep origin/at32
      6. 检查步骤 2-5 都有输出
    Expected Result: 所有新分支已推送到远程
    Failure Indicators: 任何步骤无输出
    Evidence: .sisyphus/evidence/task-9-new-branches-pushed.txt
  ```

  **Commit**: NO（此任务不创建新提交，只推送现有提交）

---
## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists (run git commands, check branch contents). For each "Must NOT Have": search for forbidden patterns. Check evidence files exist in .sisyphus/evidence/.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [x] F2. **Code Quality Review** — `unspecified-high`
  Review all changed files for: shell script quality, documentation completeness, gitignore correctness. Check for common issues: unquoted variables in scripts, missing error handling, inconsistent naming.
  Output: `Scripts [PASS/FAIL] | Docs [PASS/FAIL] | Gitignore [PASS/FAIL] | VERDICT`

- [x] F3. **Real Manual QA** — `unspecified-high`
  Start from clean state. Execute EVERY QA scenario from EVERY task — follow exact steps, capture evidence. Test cross-task integration: verify sync script works with all platform branches. Test edge cases: merge conflicts, empty branches.
  Output: `Scenarios [N/N pass] | Integration [N/N] | Edge Cases [N tested] | VERDICT`

- [x] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual git diff/log. Verify 1:1 — everything in spec was built, nothing beyond spec was built. Check "Must NOT do" compliance. Flag unaccounted changes.
  Output: `Tasks [N/N compliant] | Unaccounted [CLEAN/N files] | VERDICT`

---

## Commit Strategy

- **Task 2**: `refactor: clean master to common-only driver and filter` — 移除平台特定文件
- **Task 3**: `feat(stm32f407): create STM32F407 platform branch` — 创建平台分支
- **Task 4**: `feat(mspm0g3507): create MSPM0G3507 platform branch` — 创建平台分支
- **Task 5**: `feat(ch32): create CH32 platform branch` — 创建平台分支
- **Task 6**: `feat(at32): create AT32 platform branch` — 创建平台分支
- **Task 7**: `feat(scripts): add auto-sync script for platform branches` — 同步脚本
- **Task 8**: `docs: add branching workflow documentation` — 工作流文档
- **Task 9**: `chore: cleanup old branches and push to remote` — 清理旧分支

---

## Success Criteria

### Verification Commands
```bash
# master 分支不含平台特定文件
git ls-tree -r master --name-only | grep -E "(test_lsm6dsr|\.uvprojx)"  # Expected: no output

# 每个平台分支存在
git branch -a | grep -E "(stm32f407|mspm0g3507|ch32|at32)"  # Expected: 4 branches

# 同步脚本存在且可执行
test -x scripts/sync-to-platforms.sh  # Expected: exit 0

# 文档存在
test -f docs/BRANCHING.md  # Expected: exit 0
```

### Final Checklist
- [ ] master 分支只含通用代码
- [ ] 4 个平台分支创建完成
- [ ] 同步脚本可正常工作
- [ ] 文档描述了完整工作流
- [ ] 远程仓库已更新
