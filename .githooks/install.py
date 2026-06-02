#!/usr/bin/env python3
"""安装 git hooks

功能：
- 检测 Python 版本 >= 3.6
- 设置 git config core.hooksPath
- 显示安装成功提示
"""
import subprocess
import sys
from pathlib import Path


def check_python_version() -> None:
    """检查 Python 版本 >= 3.6"""
    if sys.version_info < (3, 6):
        print(f"❌ 需要 Python >= 3.6，当前: {sys.version}")
        sys.exit(1)


def check_git_repo() -> None:
    """检查是否在 git 仓库中"""
    try:
        subprocess.run(
            ['git', 'rev-parse', '--git-dir'],
            capture_output=True,
            check=True,
        )
    except subprocess.CalledProcessError:
        print("❌ 当前目录不是 git 仓库")
        sys.exit(1)
    except FileNotFoundError:
        print("❌ 未找到 git 命令，请确保 git 已安装并在 PATH 中")
        sys.exit(1)


def install_hooks() -> None:
    """安装 git hooks"""
    hooks_path = Path(__file__).parent.resolve()

    # 检查 hooks 目录下是否有 hook 脚本
    hook_files = list(hooks_path.glob('*.py'))
    if not hook_files:
        print(f"❌ 未找到 hook 脚本: {hooks_path}")
        sys.exit(1)

    # 设置 core.hooksPath
    subprocess.run(
        ['git', 'config', 'core.hooksPath', str(hooks_path)],
        check=True,
    )

    print(f"✅ Git hooks 已安装")
    print(f"   hooks 路径: {hooks_path}")
    print(f"   已设置: git config core.hooksPath {hooks_path}")
    print()
    print(f"   卸载: git config --unset core.hooksPath")
    print(f"   临时跳过: git commit --no-verify")


def main() -> None:
    check_python_version()
    check_git_repo()
    install_hooks()


if __name__ == '__main__':
    main()
