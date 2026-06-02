#!/usr/bin/env python3
"""校验 Conventional Commits 格式

允许的 type: feat, fix, refactor, docs, chore, test, style, perf, tune
格式: <type>(<scope>): <description>
支持 Merge commit 和 Revert 前缀（自动跳过校验）
"""
import sys
import re
from pathlib import Path

VALID_TYPES = {
    'feat', 'fix', 'refactor', 'docs', 'chore',
    'test', 'style', 'perf', 'tune'
}

PATTERN = re.compile(
    r'^(?P<type>\w+)(?:\((?P<scope>[^)]+)\))?:\s(?P<desc>.+)$'
)

SKIP_PREFIXES = ('Merge ', 'Revert ')


def main() -> int:
    if len(sys.argv) < 2:
        print("❌ 用法: commit-msg.py <commit-msg-file>")
        return 1

    msg_file = Path(sys.argv[1])
    if not msg_file.exists():
        print(f"❌ 文件不存在: {msg_file}")
        return 1

    msg = msg_file.read_text(encoding='utf-8').strip()

    # 跳过空消息（注释行）
    lines = [line for line in msg.splitlines() if not line.startswith('#')]
    msg = '\n'.join(lines).strip()

    if not msg:
        return 0

    # 跳过 Merge 和 Revert
    if msg.startswith(SKIP_PREFIXES):
        return 0

    # 匹配 Conventional Commits 格式
    first_line = msg.splitlines()[0]
    m = PATTERN.match(first_line)

    if not m:
        print("❌ 提交消息格式错误")
        print(f"   期望: <type>(<scope>): <description>")
        print(f"   示例: feat(driver): add SPI support")
        print(f"   允许的 type: {', '.join(sorted(VALID_TYPES))}")
        print(f"   scope 可选，如: docs: update README")
        return 1

    type_ = m.group('type')
    if type_ not in VALID_TYPES:
        print(f"⚠️  非标 type: '{type_}'")
        print(f"   允许的 type: {', '.join(sorted(VALID_TYPES))}")
        print(f"   如需继续，请使用 git commit --no-verify")
        return 1

    desc = m.group('desc')
    if not desc or len(desc.strip()) < 3:
        print(f"❌ description 过短: '{desc}'")
        print(f"   描述至少需要 3 个字符")
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
