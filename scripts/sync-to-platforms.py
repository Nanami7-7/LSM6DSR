#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""
sync-to-platforms.py — 同步 master 通用层到所有平台分支

用法:
    uv run scripts/sync-to-platforms.py              # 同步所有平台分支
    uv run scripts/sync-to-platforms.py --dry-run    # 预览模式，不实际执行
    uv run scripts/sync-to-platforms.py --branch stm32f407  # 只同步指定分支
    uv run scripts/sync-to-platforms.py --force      # 强制同步（丢弃平台本地修改）

功能:
    将 master 分支合并到每个平台分支（单向同步）
    平台分支只拉取 master 的通用代码，不会反向合并

注意:
    运行前确保 master 分支的工作区干净（无未提交的修改）
"""

import subprocess
import sys
import argparse
import os
from pathlib import Path
from typing import List, Optional, Tuple

# Windows 控制台编码修复
if sys.platform == 'win32':
    os.system('chcp 65001 > nul 2>&1')
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')

# === 配置 ============================================================

PLATFORM_BRANCHES = ["stm32f407", "mspm0g3507", "ch32", "at32"]
MASTER_BRANCH = "master"

# === 工具函数 ========================================================

def run_git(args: List[str], capture: bool = False) -> Tuple[int, str, str]:
    """运行 git 命令"""
    cmd = ["git"] + args
    if capture:
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    else:
        result = subprocess.run(cmd)
        return result.returncode, "", ""


def get_current_branch() -> str:
    """获取当前分支名"""
    rc, out, _ = run_git(["branch", "--show-current"], capture=True)
    return out if rc == 0 else ""


def is_working_tree_clean() -> bool:
    """检查工作区是否干净"""
    rc, out, _ = run_git(["status", "--porcelain"], capture=True)
    return rc == 0 and len(out) == 0


def branch_exists(branch: str) -> bool:
    """检查分支是否存在"""
    rc, _, _ = run_git(["rev-parse", "--verify", branch], capture=True)
    return rc == 0


def get_merge_base(branch1: str, branch2: str) -> Optional[str]:
    """获取两个分支的合并基"""
    rc, out, _ = run_git(["merge-base", branch1, branch2], capture=True)
    return out if rc == 0 else None


def count_commits(from_branch: str, to_branch: str) -> int:
    """计算从 from_branch 到 to_branch 的提交数"""
    rc, out, _ = run_git(["rev-list", "--count", f"{from_branch}..{to_branch}"], capture=True)
    return int(out) if rc == 0 else 0


def checkout_branch(branch: str) -> bool:
    """切换到指定分支"""
    rc, _, _ = run_git(["checkout", branch])
    return rc == 0


def merge_branch(branch: str, message: str = "") -> Tuple[bool, str]:
    """合并指定分支"""
    args = ["merge", branch, "--no-edit"]
    if message:
        args.extend(["-m", message])
    rc, out, err = run_git(args, capture=True)
    output = out + err
    return rc == 0, output


def abort_merge():
    """中止合并"""
    run_git(["merge", "--abort"])


def get_conflict_files() -> List[str]:
    """获取冲突文件列表"""
    rc, out, _ = run_git(["diff", "--name-only", "--diff-filter=U"], capture=True)
    return out.split("\n") if rc == 0 and out else []


# === 主逻辑 ==========================================================

def sync_branch(platform_branch: str, dry_run: bool = False, force: bool = False) -> bool:
    """同步 master 到指定平台分支"""
    print(f"\n--- 同步到 {platform_branch} ---")

    # 检查分支是否存在
    if not branch_exists(platform_branch):
        print(f"  ⚠️  分支 {platform_branch} 不存在，跳过")
        return False

    # 计算差异
    ahead = count_commits(MASTER_BRANCH, platform_branch)
    behind = count_commits(platform_branch, MASTER_BRANCH)

    print(f"  状态: {platform_branch} 领先 master {ahead} 个提交，落后 {behind} 个提交")

    if behind == 0:
        print(f"  ✅ {platform_branch} 已是最新，无需同步")
        return True

    if dry_run:
        print(f"  [DRY RUN] 将合并 master 的 {behind} 个提交到 {platform_branch}")
        return True

    # 切换到平台分支
    if not checkout_branch(platform_branch):
        print(f"  ❌ 无法切换到 {platform_branch}")
        return False

    # 强制模式：使用 reset 而不是 merge
    if force:
        print(f"  ⚠️  强制模式：重置 {platform_branch} 到 master 的状态")
        run_git(["reset", "--hard", MASTER_BRANCH])
        print(f"  ✅ {platform_branch} 已强制同步到 master")
        return True

    # 合并 master
    success, output = merge_branch(MASTER_BRANCH)

    if success:
        print(f"  ✅ {platform_branch} 合并成功")
        return True
    else:
        # 合并冲突
        conflict_files = get_conflict_files()
        print(f"  ❌ {platform_branch} 合并冲突！")
        print(f"  冲突文件:")
        for f in conflict_files:
            print(f"    - {f}")

        # 中止合并，恢复原状态
        abort_merge()
        print(f"  已自动中止合并，请手动解决冲突后重试")
        return False


def main():
    parser = argparse.ArgumentParser(description="同步 master 到平台分支（单向同步）")
    parser.add_argument("--dry-run", action="store_true", help="预览模式，不实际执行")
    parser.add_argument("--branch", type=str, help="只同步指定分支")
    parser.add_argument("--force", action="store_true", help="强制同步（丢弃平台本地修改）")
    parser.add_argument("--all", action="store_true", help="同步所有分支（包括不存在的）")
    args = parser.parse_args()

    print("=== LSM6DSR 平台同步脚本 ===")
    print(f"源分支: {MASTER_BRANCH}")
    print(f"目标分支: {', '.join(PLATFORM_BRANCHES)}")

    if args.dry_run:
        print("\n[DRY RUN 模式] 不会实际执行任何操作")

    if args.force:
        print("\n[强制模式] 将丢弃平台分支的本地修改")

    # 检查工作区是否干净
    if not args.dry_run and not is_working_tree_clean():
        print("\n❌ 错误: 工作区有未提交的修改，请先 commit 或 stash")
        sys.exit(1)

    # 保存当前分支
    original_branch = get_current_branch()
    if not original_branch:
        print("\n❌ 错误: 无法获取当前分支")
        sys.exit(1)

    print(f"\n当前分支: {original_branch}")

    # 确定要同步的分支
    if args.branch:
        if args.branch not in PLATFORM_BRANCHES:
            print(f"\n❌ 错误: {args.branch} 不是平台分支")
            print(f"可用的平台分支: {', '.join(PLATFORM_BRANCHES)}")
            sys.exit(1)
        branches_to_sync = [args.branch]
    else:
        branches_to_sync = PLATFORM_BRANCHES

    # 切换到 master（如果不是的话）
    if original_branch != MASTER_BRANCH and not args.dry_run:
        if not checkout_branch(MASTER_BRANCH):
            print(f"\n❌ 错误: 无法切换到 {MASTER_BRANCH}")
            sys.exit(1)

    # 执行同步
    success_count = 0
    fail_count = 0

    for branch in branches_to_sync:
        if sync_branch(branch, dry_run=args.dry_run, force=args.force):
            success_count += 1
        else:
            fail_count += 1

    # 返回原分支
    if not args.dry_run and get_current_branch() != original_branch:
        checkout_branch(original_branch)

    # 打印总结
    print("\n=== 同步完成 ===")
    print(f"成功: {success_count} 个分支")
    if fail_count > 0:
        print(f"失败: {fail_count} 个分支")

    if not args.dry_run and success_count > 0:
        print(f"\n下一步: git push origin <branch> 推送各平台分支")
        print("  git push origin stm32f407 mspm0g3507 ch32 at32")

    sys.exit(0 if fail_count == 0 else 1)


if __name__ == "__main__":
    main()
