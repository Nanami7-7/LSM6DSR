# Git Hooks

基于 Python 的 git hook 系统，用于强制执行代码提交规范。

## 安装

```bash
python .githooks/install.py
```

安装后，git 将自动在提交时运行 hook 检查。

## 功能

### commit-msg — 提交消息校验

强制使用 [Conventional Commits](https://www.conventionalcommits.org/) 格式：

```
<type>(<scope>): <description>
```

**允许的 type：**
- `feat` — 新功能
- `fix` — 修复 bug
- `refactor` — 重构（不改变功能）
- `docs` — 文档更新
- `chore` — 构建/工具/依赖
- `test` — 测试相关
- `style` — 代码格式（不影响逻辑）
- `perf` — 性能优化
- `tune` — 参数调优

**示例：**
```
feat(driver): add SPI support
fix(parser): handle empty input
docs: update README
tune(filter): adjust alpha to 0.8
```

**特殊规则：**
- `scope` 可选，如 `docs: update README`
- `description` 至少 3 个字符
- `Merge commit` 和 `Revert` 前缀自动跳过校验

### pre-commit — 提交前检查

| 检查项 | 阈值 | 行为 |
|--------|------|------|
| 大文件 | >500KB | 警告 |
| 大文件 | >2MB | 阻止提交 |
| 二进制文件 | .exe, .dll, .so, .dylib | 阻止提交 |
| 调试代码 | printf("DEBUG, console.log, breakpoint() | 警告（不阻止） |

## 卸载

```bash
git config --unset core.hooksPath
```

## 临时跳过

```bash
git commit --no-verify
```

## 跨平台兼容

所有脚本使用 Python 标准库，兼容 Windows、macOS 和 Linux。

要求：Python >= 3.6
