# Issues

## Task 9: 清理旧分支并推送到远程仓库 (2026-06-03)

### 问题描述
- **错误**: `remote: Permission to Nanami7-7/LSM6DSR.git denied to loopgad.`
- **HTTP 状态码**: 403 (Forbidden)
- **原因**: 当前认证用户 `loopgad` 没有写入 `Nanami7-7/LSM6DSR.git` 仓库的权限

### 已完成的本地操作
- ✅ 删除本地分支 `demo/stm32f407`
- ✅ 删除本地分支 `agents/refactor-lsm6dsr-common-driver`
- ✅ 清理 prunable worktree

### 受阻的远程操作
- ❌ 推送 master 到远程
- ❌ 推送 stm32f407 到远程
- ❌ 推送 mspm0g3507 到远程
- ❌ 推送 ch32 到远程
- ❌ 推送 at32 到远程
- ❌ 推送分支删除到远程 (demo/stm32f407, agents/refactor-lsm6dsr-common-driver)

### 解决方案
1. **检查 GitHub 认证**:
   ```bash
   git credential-manager erase  # 清除旧凭据
   git push origin master  # 重新输入凭据
   ```

2. **使用 SSH 方式** (推荐):
   ```bash
   git remote set-url origin git@github.com:Nanami7-7/LSM6DSR.git
   git push origin master
   ```

3. **检查仓库权限**:
   - 确认 `loopgad` 账户是否有 `Nanami7-7/LSM6DSR` 仓库的写入权限
   - 如果是 fork 仓库，需要 push 到自己的 fork

4. **使用个人访问令牌 (PAT)**:
   ```bash
   git remote set-url origin https://<username>:<token>@github.com/Nanami7-7/LSM6DSR.git
   ```
