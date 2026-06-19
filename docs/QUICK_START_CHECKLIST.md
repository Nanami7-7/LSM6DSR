# LSM6DSR 项目快速开始实施清单

## 🚀 立即执行（今天）

### 1. 提交当前改进
```bash
cd D:/msp_project/LSM6DSR/LSM6DSR

# 检查状态
git status

# 提交所有改进
git add -A
git commit -m "refactor: improve filter library architecture and add platform abstraction

- Reorganize filter library to Core/Filter/ directory
- Add filter chain support
- Add platform abstraction layer
- Improve logging system
- Add comprehensive test framework"

# 创建版本标签
git tag -a v1.0.0 -m "Release v1.0.0: Complete filter library with 6 algorithms"

# 推送到远程
git push origin master --tags
```

### 2. 创建 `.gitignore` 改进
```bash
cat >> .gitignore << 'EOF'

# Build artifacts
*.o
*.obj
*.elf
*.hex
*.bin
*.exe
*.out
*.d

# IDE specific
.vscode/
.idea/
*.swp
*.swo
*~

# OS specific
.DS_Store
Thumbs.db

# Test coverage
*.gcda
*.gcno
*.gcov
coverage/

# Documentation build
docs/api/
!docs/api/.gitkeep
EOF

git add .gitignore
git commit -m "chore: update .gitignore with comprehensive rules"
```

### 3. 添加代码风格配置
```bash
cat > .clang-format << 'EOF'
BasedOnStyle: LLVM
IndentWidth: 4
TabWidth: 4
UseTab: Never
ColumnLimit: 100
AllowShortFunctionsOnASingleLine: None
AllowShortIfStatementsOnASingleLine: false
AllowShortLoopsOnASingleLine: false
BreakBeforeBraces: Linux
SortIncludes: false
PointerAlignment: Right
SpaceBeforeParens: ControlStatements
EOF

git add .clang-format
git commit -m "style: add clang-format configuration for consistent code style"
```

---

## 📋 本周计划

### Day 1-2: 分支结构
```bash
# 创建 STM32F407 平台分支
git checkout -b stm32f407
# 复制 STM32 相关文件（如果有的话）
git push -u origin stm32f407

# 创建 MSPM0G3507 分支
git checkout master
git checkout -b mspm0g3507
git push -u origin mspm0g3507

# 创建 CH32 分支
git checkout master
git checkout -b ch32
git push -u origin ch32

# 创建 AT32 分支
git checkout master
git checkout -b at32
git push -u origin at32

# 回到 master
git checkout master
```

### Day 3-4: CI/CD 基础
```bash
mkdir -p .github/workflows
```

创建 `.github/workflows/ci.yml`（见 IMPROVEMENT_PLAN.md）

### Day 5-7: 基本示例
```bash
mkdir -p examples/basic_usage
# 创建 examples/basic_usage/main.c（见 IMPROVEMENT_PLAN.md）

mkdir -p examples/filter_comparison
# 创建滤波器性能对比示例

git add examples/
git commit -m "docs: add basic usage examples for all filter types"
```

---

## 🔍 使用 CodeGraph 分析项目

现在让我用 CodeGraph 来深入分析代码结构：

```bash
# CodeGraph 已经在项目中初始化，可以直接使用

# 查找所有函数定义
# codegraph_search: "函数名"

# 查看函数调用关系
# codegraph_callers: "filter_create"
# codegraph_callees: "bsp_lsm6dsr_init"

# 获取代码上下文
# codegraph_context: "filter_update"
```

---

## 🧪 验证测试

```bash
# 编译并运行单元测试
cd D:/msp_project/LSM6DSR/LSM6DSR/test

# 编译测试
gcc -o test_filters test_filters.c \
    ../Core/Filter/Src/filter.c \
    ../Core/Filter/Src/filter_chain.c \
    ../Core/Filter/Src/filter_config.c \
    -I../Core/Filter/Inc \
    -I../Core/Inc \
    -lm -Wall -Wextra

# 运行测试
./test_filters

# 查看测试结果
# 预期：137/137 测试通过
```

---

## 📊 使用 Superpowers 技能

### 1. 头脑风暴（Brainstorming）
如何改进滤波器性能？

### 2. 系统化调试（Systematic Debugging）
如果测试失败，系统化排查：
1. 重现问题
2. 隔离原因
3. 验证修复

### 3. 测试驱动开发（TDD）
添加新功能时：
1. 先写测试
2. 实现功能
3. 重构代码

### 4. 代码审查（Code Review）
提交 PR 前：
1. 自检清单
2. 请求审查
3. 处理反馈

---

## ✅ 每日检查清单

- [ ] 代码编译无警告
- [ ] 所有测试通过
- [ ] 提交消息符合规范
- [ ] 文档已更新（如有 API 变更）
- [ ] 代码风格符合规范

---

## 📚 相关文档

- `README.md` - 项目概述
- `docs/BRANCHING.md` - 分支管理规范
- `docs/IMPROVEMENT_PLAN.md` - 完整改进计划
- `Doc/hardware_wiring.md` - 硬件接线
- `docs/bsp_tuning_guide.md` - BSP 调参指南

---

## 🎯 里程碑

### v1.0.0（当前）
- ✅ 核心驱动层
- ✅ 滤波器库（6 种算法）
- ✅ 基本文档

### v1.1.0（1 周后）
- [ ] 所有平台分支
- [ ] CI/CD 基础
- [ ] 基本示例

### v1.2.0（1 个月后）
- [ ] 完整测试覆盖
- [ ] API 文档自动生成
- [ ] 性能基准测试

### v2.0.0（3 个月后）
- [ ] 高级功能（运动检测、校准）
- [ ] 生产级质量
- [ ] 社区贡献流程

---

## 💡 提示

1. **小步提交**：每个功能/修复一个提交
2. **及时同步**：定期同步 master 到平台分支
3. **测试先行**：新功能先写测试
4. **文档同步**：API 变更必须更新文档

---

**开始执行吧！** 🚀

从上面的"立即执行"部分开始，逐步完成每个阶段。
