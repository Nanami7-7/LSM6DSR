# LSM6DSR 分支管理工作流

## 1. 分支策略概述

本项目采用 **通用层 + 平台分支** 的分支模型。核心思路：

- `master` 分支只维护**平台无关的通用代码**（驱动层 + BSP 层 + ST 寄存器定义）
- 每个硬件平台一个独立分支，包含**平台特定的 HAL 配置、启动文件、外设驱动、测试代码**
- 通用层更新通过脚本自动同步到所有平台分支

这种设计让不同平台的开发者互不干扰，同时保证通用驱动的 bugfix 能快速同步到所有平台。

---

## 2. 分支结构

```
master (通用层)
├── Core/Src/lsm6dsr.c          # 驱动层
├── Core/Src/bsp_lsm6dsr.c      # 业务层
├── Core/Inc/lsm6dsr.h          # 驱动头文件
├── Core/Inc/bsp_lsm6dsr.h      # 业务头文件
├── lsm6dsr_STdC/               # ST 官方寄存器定义
├── ports/                      # 平台桩文件目录
│   ├── mspm0g3507/
│   ├── ch32/
│   └── at32/
├── scripts/                    # 自动化脚本
└── docs/                       # 文档

stm32f407 (STM32F407 平台)
├── [继承 master 所有通用文件]
├── Core/Src/main.c             # STM32 主程序
├── Core/Src/test_lsm6dsr.c     # STM32 测试层
├── Core/Src/stm32f4xx_it.c     # 中断处理
├── Core/Src/i2c.c, gpio.c ...  # CubeMX 生成
├── Drivers/                    # STM32 HAL 库
└── MDK-ARM/                    # Keil 工程文件

mspm0g3507 (MSPM0G3507 平台)
├── [继承 master 所有通用文件]
└── [平台特定文件]

ch32 (CH32 平台)
├── [继承 master 所有通用文件]
└── [平台特定文件]

at32 (AT32 平台)
├── [继承 master 所有通用文件]
└── [平台特定文件]
```

### 通用核心文件（所有分支共享）

| 文件 | 层级 | 说明 |
|------|------|------|
| `Core/Src/lsm6dsr.c` | 驱动层 | 寄存器读写、ACC/GYRO/TEMP 读取、FIFO、自检 |
| `Core/Inc/lsm6dsr.h` | 驱动层 | 寄存器映射、I/O 抽象 `lsm6dsr_io_t`、枚举 |
| `Core/Src/bsp_lsm6dsr.c` | 业务层 | 互补滤波、偏置校准、静止检测、VOFA+ 格式化 |
| `Core/Inc/bsp_lsm6dsr.h` | 业务层 | 配置宏、数据结构、生产 API 声明 |
| `lsm6dsr_STdC/` | 参考 | ST 官方 LSM6DSR Standard C 驱动 |

### 平台特定文件（仅在对应分支）

| 文件 | 说明 |
|------|------|
| `Core/Src/main.c` | 主程序入口（含平台 HAL 初始化） |
| `Core/Src/test_lsm6dsr.c` | 测试层（平台 I2C 桥接 + P1~P19 测试） |
| `Core/Inc/test_lsm6dsr.h` | 测试层头文件 |
| `Drivers/` | 平台 HAL/外设库 |
| `MDK-ARM/` 或对应 IDE 工程 | 编译工程文件 |
| CubeMX 生成的外设初始化文件 | `i2c.c`, `gpio.c`, `usart.c` 等 |

---

## 3. 新平台开发流程

以添加 AT32 平台为例：

### 3.1 从 master 创建平台分支

```bash
git checkout master
git checkout -b at32
```

### 3.2 创建平台桩文件

在 `ports/at32/` 下创建桩文件，提供函数签名和 TODO 注释：

- `test_lsm6dsr.c` — I2C 桥接函数桩（定义 `lsm6dsr_io` 实例）
- `test_lsm6dsr.h` — 测试层头文件桩
- `README.md` — 平台实现说明

桩文件设计原则：
- 不复制 STM32 代码，只提供函数签名
- I2C 读写函数返回 -1（未实现），避免链接错误
- 保留 `lsm6dsr_io` 全局实例，BSP 层可正常编译

### 3.3 添加平台 HAL 库和工程文件

```bash
# 添加平台 HAL/外设库
git add Drivers/

# 添加 IDE 工程文件
git add MDK-ARM/  # 或 Keil/IAR/GCC 工程目录

# 添加 CubeMX 或手写的外设初始化
git add Core/Src/i2c.c Core/Src/gpio.c ...
```

### 3.4 实现平台 I2C 桥接

在 `Core/Src/test_lsm6dsr.c` 中实现平台特定的 I2C 读写函数：

```c
// 示例：AT32 平台
static int32_t at32_i2c_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // 使用 AT32 I2C API 实现
    // return i2c_transfer7(...);
    return -1; // 未实现
}
```

然后将函数指针赋给 `lsm6dsr_io`：

```c
lsm6dsr_io_t lsm6dsr_io = {
    .handle = &hi2c1,
    .read   = at32_i2c_read,
    .write  = at32_i2c_write,
};
```

### 3.5 提交并推送

```bash
git add -A
git commit -m "feat(at32): add platform stub files and HAL integration"
git push origin at32
```

### 3.6 更新 ports/ 桩文件（回到 master）

```bash
git checkout master
git add ports/at32/
git commit -m "feat(ports): add AT32 platform stub files"
```

---

## 4. 通用层更新流程

当修改了 `master` 上的通用核心文件（驱动层或 BSP 层），需要同步到所有平台分支。

### 4.1 手动同步单个平台

```bash
# 在 master 上提交修改
git checkout master
git add -A
git commit -m "fix: correct gyro bias tracking on Z-axis"

# 同步到 stm32f407
git checkout stm32f407
git merge master --no-edit

# 如果有冲突，解决后继续
# git add <resolved-files>
# git commit --no-edit

git checkout master
```

### 4.2 使用脚本批量同步

项目提供了 `scripts/sync-to-platforms.sh` 脚本，自动同步到所有平台分支：

```bash
# 确保在 master 分支且工作区干净
git checkout master
git status  # 确认无未提交修改

# 执行同步
./scripts/sync-to-platforms.sh
```

脚本会依次切换到 `stm32f407`、`mspm0g3507`、`ch32`、`at32` 并执行 `git merge master`。如果某个分支出现冲突，脚本会暂停并输出冲突文件列表。

### 4.3 同步后推送所有分支

```bash
git push origin master
git push origin stm32f407 mspm0g3507 ch32 at32
```

### 4.4 典型场景

**场景 1：修复驱动 bug**

```bash
git checkout master
# 编辑 Core/Src/lsm6dsr.c
git add Core/Src/lsm6dsr.c Core/Inc/lsm6dsr.h
git commit -m "fix: handle FIFO overflow in watermark mode"
git push origin master
./scripts/sync-to-platforms.sh
git push origin stm32f407 mspm0g3507 ch32 at32
```

**场景 2：BSP 层新增功能**

```bash
git checkout master
# 编辑 Core/Src/bsp_lsm6dsr.c
git add Core/Src/bsp_lsm6dsr.c Core/Inc/bsp_lsm6dsr.h
git commit -m "feat: add quaternion output to BSP layer"
git push origin master
./scripts/sync-to-platforms.sh
```

**场景 3：更新文档**

```bash
git checkout master
# 编辑 docs/ 下的文件
git add docs/
git commit -m "docs: update tuning guide for new alpha parameters"
git push origin master
# 文档更新通常不需要同步到平台分支，除非平台分支也有独立文档
```

---

## 5. 冲突解决指南

冲突通常发生在以下场景：
- 平台分支修改了某个通用文件，master 也修改了同一文件
- 平台分支添加了同名但内容不同的文件

### 5.1 识别冲突

```bash
git status
# 输出会显示：
# Unmerged paths:
#   both modified:   Core/Src/bsp_lsm6dsr.c
```

### 5.2 解决策略

| 冲突类型 | 处理方式 |
|----------|----------|
| 平台未修改通用文件 | 直接采用 master 版本：`git checkout master -- <file>` |
| 平台有独立修改 | 手动合并，保留两边的改动 |
| 平台桩文件冲突 | 以平台分支为准，通用层不应修改桩文件 |

### 5.3 解决步骤

```bash
# 1. 查看冲突内容
git diff Core/Src/bsp_lsm6dsr.c

# 2. 编辑文件，删除冲突标记（<<<<<< / ====== / >>>>>>）
#    保留需要的代码

# 3. 标记冲突已解决
git add Core/Src/bsp_lsm6dsr.c

# 4. 完成合并
git commit --no-edit

# 5. 返回 master 继续同步其他分支
git checkout master
./scripts/sync-to-platforms.sh
```

### 5.4 预防冲突

- **不要在平台分支上修改通用核心文件**。如果需要修改，回到 master 操作。
- **平台桩文件只在 master 的 `ports/` 目录维护**，不要在平台分支上直接修改 `ports/` 下的文件。
- **定期同步**。master 有更新就尽快同步到平台分支，避免差异累积。

---

## 6. 提交规范

本项目使用 [Conventional Commits](https://www.conventionalcommits.org/) 规范。

### 6.1 格式

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

### 6.2 类型 (type)

| 类型 | 说明 | 示例 |
|------|------|------|
| `feat` | 新功能 | `feat: add FIFO watermark support` |
| `fix` | Bug 修复 | `fix: correct gyro bias tracking on Z-axis` |
| `refactor` | 重构（不改变功能） | `refactor: simplify variance calculation` |
| `docs` | 文档更新 | `docs: add branching workflow guide` |
| `chore` | 构建/工具/依赖 | `chore: update CubeMX config` |
| `test` | 测试相关 | `test: add P20 temperature accuracy test` |
| `style` | 代码格式（不影响逻辑） | `style: fix indentation in bsp_lsm6dsr.c` |
| `perf` | 性能优化 | `perf: reduce I2C transaction count` |

### 6.3 范围 (scope)

可选，用于说明影响的模块：

- `driver` — 驱动层 (`lsm6dsr.c/h`)
- `bsp` — 业务层 (`bsp_lsm6dsr.c/h`)
- `test` — 测试层
- `ports` — 平台桩文件
- `scripts` — 自动化脚本
- `docs` — 文档

示例：
```
fix(driver): handle WHO_AM_I check failure
feat(bsp): add temperature compensation
docs(ports): update AT32 implementation guide
```

### 6.4 提交消息示例

```bash
# 好的提交消息
git commit -m "fix: correct gyro bias tracking on Z-axis"
git commit -m "feat(bsp): add quaternion output"
git commit -m "docs: add branching workflow documentation"
git commit -m "refactor(driver): simplify register read/write functions"
git commit -m "test: add P20 temperature accuracy test"
git commit -m "chore: sync CubeMX generated files"

# 不好的提交消息
git commit -m "update"           # 太模糊
git commit -m "fix bug"          # 没说修了什么
git commit -m "WIP"              # 不应提交半成品
git commit -m "asdf"             # 无意义
```

### 6.5 平台特定提交

当提交只影响某个平台时，在 scope 中标注平台名：

```bash
git commit -m "feat(stm32f407): add DMA I2C transfer"
git commit -m "fix(at32): correct I2C clock configuration"
git commit -m "chore(mspm0g3507): update SDK to v2.0"
```

---

## 7. 快速参考

### 常用命令

```bash
# 查看当前分支
git branch --show-current

# 切换分支
git checkout stm32f407

# 创建新平台分支
git checkout master
git checkout -b new_platform

# 同步通用层到所有平台
./scripts/sync-to-platforms.sh

# 推送所有分支
git push origin master stm32f407 mspm0g3507 ch32 at32

# 查看分支差异
git diff master..stm32f407 --stat
```

### 分支状态检查

```bash
# 检查 master 是否有未同步的提交
git log --oneline stm32f407..master

# 检查平台分支是否有独立提交
git log --oneline master..stm32f407

# 查看通用文件在各分支的差异
git diff master..stm32f407 -- Core/Src/lsm6dsr.c
```
