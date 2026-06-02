#!/usr/bin/env python3
"""轻量 pre-commit 检查

功能：
- 大文件检测（>500KB 警告，>2MB 阻止）
- 二进制文件误提交检测（.exe, .dll, .so, .dylib）
- 调试代码残留检测（printf("DEBUG, console.log, breakpoint()）— 仅警告不阻止
"""
import subprocess
import sys
from pathlib import Path

# 阈值配置
LARGE_FILE_WARN_KB = 500
LARGE_FILE_BLOCK_KB = 2048

# 危险二进制扩展名
BINARY_EXTENSIONS = {'.exe', '.dll', '.so', '.dylib', '.bin', '.o', '.obj'}

# 调试代码模式（仅警告）
DEBUG_PATTERNS = [
    'printf("DEBUG',
    'console.log(',
    'breakpoint()',
    'debugger;',
    'import pdb',
    'pdb.set_trace()',
]


def run_git(args: list[str]) -> str:
    """运行 git 命令并返回 stdout"""
    result = subprocess.run(
        ['git'] + args,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def get_staged_files() -> list[str]:
    """获取暂存区文件列表"""
    output = run_git(['diff', '--cached', '--name-only', '--diff-filter=ACM'])
    return [f for f in output.splitlines() if f]


def check_large_files(files: list[str]) -> int:
    """检查大文件：>2MB 阻止，>500KB 警告"""
    warnings = 0
    for f in files:
        try:
            output = run_git(['cat-file', '-s', f':{f}'])
            size = int(output)
            size_kb = size // 1024

            if size_kb > LARGE_FILE_BLOCK_KB:
                print(f"❌ 文件过大: {f} ({size_kb}KB > {LARGE_FILE_BLOCK_KB}KB)")
                print(f"   请使用 Git LFS 或减小文件大小")
                return 1
            elif size_kb > LARGE_FILE_WARN_KB:
                print(f"⚠️  大文件警告: {f} ({size_kb}KB)")
                warnings += 1
        except (ValueError, subprocess.SubprocessError):
            pass

    if warnings:
        print(f"   共 {warnings} 个大文件警告（不阻止提交）")
    return 0


def check_binary_files(files: list[str]) -> int:
    """检查二进制文件误提交"""
    for f in files:
        ext = Path(f).suffix.lower()
        if ext in BINARY_EXTENSIONS:
            print(f"❌ 二进制文件不应提交: {f}")
            print(f"   请使用 Git LFS 或添加到 .gitignore")
            return 1
    return 0


def check_debug_code(files: list[str]) -> int:
    """检查调试代码残留（仅警告，不阻止）"""
    warnings = 0
    # 只检查源代码文件
    checkable_extensions = {'.c', '.h', '.py', '.js', '.ts', '.cpp', '.hpp'}

    for f in files:
        ext = Path(f).suffix.lower()
        if ext not in checkable_extensions:
            continue

        try:
            output = run_git(['show', f':{f}'])
            for i, line in enumerate(output.splitlines(), 1):
                for pattern in DEBUG_PATTERNS:
                    if pattern in line:
                        print(f"⚠️  调试代码: {f}:{i} — {line.strip()[:60]}")
                        warnings += 1
        except subprocess.SubprocessError:
            pass

    if warnings:
        print(f"   共 {warnings} 处调试代码（不阻止提交）")
    return 0


def main() -> int:
    files = get_staged_files()
    if not files:
        return 0

    print(f"🔍 检查 {len(files)} 个文件...")

    rc = 0
    rc |= check_large_files(files)
    rc |= check_binary_files(files)
    check_debug_code(files)  # 仅警告，不影响返回值

    if rc == 0:
        print("✅ Pre-commit 检查通过")

    return rc


if __name__ == '__main__':
    sys.exit(main())
