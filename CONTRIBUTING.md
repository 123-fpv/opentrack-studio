# Contributing / 参与开发

Everyone is welcome to fork OpenTrack Studio, change the code, improve the
Chinese/English UI, report recognition failures, and submit a pull request.
This public repository does not require a paid account or an application login.
Use a fork and pull request for changes; public visibility does not grant direct
write access to the maintained branch.

## Development

Use Windows x64, Visual Studio 2022 C++ tools, CMake and Qt 6.8.3 MSVC x64.

```powershell
cmake -S . -B D:/CodexWork/OpenTrackStudio/build -DCMAKE_PREFIX_PATH="C:/Qt/6.8.3/msvc2022_64"
cmake --build D:/CodexWork/OpenTrackStudio/build --config Release
ctest --test-dir D:/CodexWork/OpenTrackStudio/build -C Release --output-on-failure
```

Keep a change focused. Explain the concrete problem, resulting behavior and
validation in your PR. Include a regression for a semantic or geometry bug;
small wording edits generally do not need new tests. CI runs on public pull
requests with read-only repository permissions and no service secrets.

For recognition bugs, include the gate/step number, entry face, exit face,
wrapping side and expected order. Prefer a small original synthetic diagram.
Only attach photos, diagrams or tracks you have permission to publish. A gate
count alone cannot establish that the route is correct. Do not replace an
observed progressive wrap with a closed orbit or fill hidden apertures merely
because the component has that shape.

Keep language-neutral IDs, project JSON fields and simulator records unchanged
when adding translations. Translate UI text and messages, not user names,
file paths or track data. Check both languages and preserve unsaved edits and
undo history when switching language.

Never commit API keys, credentials, local settings, private diagrams, simulator
databases, game assets or generated build directories. The application stores
optional credentials in Windows Credential Manager, outside track projects.

## Licensing

Contributions to the current application are accepted under **GPL-3.0-or-later**,
the same project terms described in [LICENSE](LICENSE) and
[NOTICE](NOTICE). Submit only work you may license this way; retain any required
third-party notices. Copyright remains with the applicable contributors;
submitting a PR does not assign your copyright to the maintainer.

The MIT terms on versions through 0.1.6 remain in their release history and the
preserved notice. They do not make new GPL-covered contributions MIT licensed.

## 中文

欢迎任何人 Fork、修改代码、补充中英文翻译、报告识别问题并提交 Pull Request。
公开仓库允许共同参与，主分支仍通过 PR 审阅合入，不是向所有人开放直接写入权限。

提交时说明具体问题、修改后的行为和验证结果。识别问题请指出门编号、步骤顺序、
进入面、穿出面和绕行侧，优先提供可以公开的简单自绘图。不能仅凭门型补齐未经过的
窗口，也不能用原窗口闭环整圈替代图中的连续绕杆换层。

翻译不修改工程字段、对象 ID 或用户内容，切换语言需保留未保存修改及撤销记录。
禁止提交密钥、凭据、私人图片、数据库、模拟器资产及构建目录。

新贡献使用 GPL-3.0-or-later，提交前请确认拥有相应授权，并保留第三方声明。
