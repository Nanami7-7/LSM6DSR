# Scripts

LSM6DSR 项目自动化脚本集合。

## 脚本列表

| 脚本 | 用途 |
|------|------|
| `sync-to-platforms.sh` | 同步 master 通用层到所有平台分支 |

## sync-to-platforms.sh

### 功能

将 `master` 分支的通用核心代码（驱动层 + BSP 层）同步到所有平台分支：

- `stm32f407` — STM32F407 平台
- `mspm0g3507` — MSPM0G3507 平台
- `ch32` — CH32 平台
- `at32` — AT32 平台

### 用法

```bash
# 确保在 master 分支且工作区干净
git checkout master
git status  # 确认无未提交修改

# 执行同步
./scripts/sync-to-platforms.sh
```

### 工作流程

1. 检查工作区是否干净（有未提交修改则报错退出）
2. 遍历 4 个平台分支
3. 切换到平台分支，执行 `git merge master --no-edit`
4. 合并成功 → 继续下一个分支
5. 合并冲突 → 暂停，输出冲突文件列表，提示手动解决

### 冲突处理

如果遇到冲突：

```bash
# 1. 查看冲突文件
git status

# 2. 手动解决冲突（编辑文件，删除冲突标记）

# 3. 标记冲突已解决
git add <resolved-files>

# 4. 完成合并
git commit --no-edit

# 5. 返回 master
git checkout master

# 6. 重新运行脚本（会跳过已完成的分支）
./scripts/sync-to-platforms.sh
```

### 注意事项

- **运行前必须在 master 分支** 且工作区干净
- 脚本使用 `set -euo pipefail`，任何错误会立即终止
- 冲突时脚本不会自动解决，需要手动处理
- 建议先 `git push origin master` 再同步，方便回溯
- 每个平台分支合并后会立即切换到下一个分支

### 典型使用场景

```bash
# 场景 1: 修改了通用核心文件（lsm6dsr.c/h, bsp_lsm6dsr.c/h）

git checkout master
# ... 编辑文件 ...
git add -A && git commit -m "fix: update driver logic"
git push origin master

# 同步到所有平台
./scripts/sync-to-platforms.sh

# 推送所有平台分支
git push origin stm32f407 mspm0g3507 ch32 at32
```

```bash
# 场景 2: 新增通用头文件

git checkout master
# ... 添加新文件 ...
git add -A && git commit -m "feat: add new common header"
./scripts/sync-to-platforms.sh
```
