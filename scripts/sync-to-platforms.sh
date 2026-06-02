#!/bin/bash
# sync-to-platforms.sh — 同步 master 通用层到所有平台分支
#
# 用法: ./scripts/sync-to-platforms.sh
# 功能: 将 master 分支合并到每个平台分支，冲突时暂停提示用户手动解决
# 注意: 运行前确保工作区干净（无未提交的修改）

set -euo pipefail

PLATFORM_BRANCHES=("stm32f407" "mspm0g3507" "ch32" "at32")
MASTER_BRANCH="master"

echo "=== LSM6DSR 平台同步脚本 ==="
echo "源分支: $MASTER_BRANCH"
echo "目标分支: ${PLATFORM_BRANCHES[*]}"
echo ""

# 检查工作区是否干净
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "错误: 工作区有未提交的修改，请先 commit 或 stash"
    exit 1
fi

# 保存当前分支
CURRENT_BRANCH=$(git branch --show-current)

# 检查 master 分支是否有未推送的提交
MASTER_AHEAD=$(git rev-list --count origin/$MASTER_BRANCH..$MASTER_BRANCH 2>/dev/null || echo "0")
if [ "$MASTER_AHEAD" -gt 0 ]; then
    echo "注意: $MASTER_BRANCH 分支有 $MASTER_AHEAD 个未推送的提交"
    echo "建议先执行 git push origin $MASTER_BRANCH"
    echo ""
fi

SYNC_SUCCESS=0
SYNC_FAIL=0

for branch in "${PLATFORM_BRANCHES[@]}"; do
    echo "--- 同步到 $branch ---"
    
    # 检查分支是否存在
    if ! git rev-parse --verify "$branch" >/dev/null 2>&1; then
        echo "警告: 分支 $branch 不存在，跳过"
        ((SYNC_FAIL++))
        continue
    fi
    
    git checkout "$branch"
    
    if git merge "$MASTER_BRANCH" --no-edit; then
        echo "✅ $branch 合并成功"
        ((SYNC_SUCCESS++))
    else
        echo ""
        echo "❌ $branch 合并冲突！"
        echo "   冲突文件:"
        git diff --name-only --diff-filter=U | sed 's/^/     /'
        echo ""
        echo "   请手动解决冲突后执行:"
        echo "     git add <resolved-files>"
        echo "     git commit --no-edit"
        echo "     git checkout $CURRENT_BRANCH"
        echo "   然后重新运行本脚本继续同步剩余分支"
        echo ""
        # 保持在冲突分支，不自动返回
        exit 1
    fi
done

# 返回原分支
git checkout "$CURRENT_BRANCH"

echo ""
echo "=== 同步完成 ==="
echo "成功: $SYNC_SUCCESS 个分支"
if [ "$SYNC_FAIL" -gt 0 ]; then
    echo "跳过: $SYNC_FAIL 个分支（不存在）"
fi
echo ""
echo "下一步: git push origin <branch> 推送各平台分支"
