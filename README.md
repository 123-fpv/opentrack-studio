# OpenTrack Studio

<img src="assets/app-icon.svg" width="72" alt="OpenTrack Studio 应用图标">

用图片搭建 FPV 赛道的 C++ 桌面编辑器。**0.1.3 预览版**可在本地解析深蓝轮廓、洋红路线的俯视设计图，识别组合门并参考绘制的路线排列顺序；也保留简单蓝色门架图识别。在默认的 3D 场景中校对位置、高度和通过顺序，再导出供 VelociDrone 导入的 `.trk` 文件。

**无需应用账号登录，工程保存在本机。** 平面图识别完全在本地运行；可选的实拍 AI 功能仅在点击“生成草稿”后，将选定照片发送到 DeepSeek 官方 API。界面、图标、示例赛道和实现代码独立编写，源码采用 MIT 许可证。

![OpenTrack Studio 默认 3D 编辑场景](docs/studio-preview.png)

## 当前功能

- 黑白高对比界面，内置操作说明；默认打开可编辑的 3D 场景，可旋转视角、平移、缩放，也可切换到俯视图。
- 把单张 PNG / JPG / BMP 拖到左侧小框、3D 场景或俯视图，自动在本地生成草稿；一次撤销恢复导入前的整个工程。
- 深蓝轮廓配洋红路线的设计图支持标准门、双联门、三联门、叠门、立方门和双层立方门，识别时尝试排除图例和起飞台等干扰。
- 可选实拍/航拍 AI 草稿：最多选择 8 张照片，补充已知门宽及起点、方向、门序说明，预览确认后再应用。
- 在场景中摆放、选择、拖动、旋转门架，调整尺寸和中心高度。
- 地面使用 X / Y 坐标，Z 向上。标准门及组合门每格横、竖中心距 3 m，杆长 3.2 m、截面 0.1 × 0.1 m；转向柱默认同截面、长 3.2 m。
- 组合门可选择通过的列、层；组件可独立选颜色，黑白界面保持简洁。
- 选中门架后显示属性；比例和底图高级设置默认收起。
- 按通过顺序查看门架，显示平滑路线和几何路径长度。
- 将完整工程保存为 `.otproj`，底图随工程嵌入。
- 导入、导出 VelociDrone `.trk`；新建方梁标准门、起终点门、转向柱及上述组合门均支持导出。

本地识别面向清晰、平整且符合上述样式的设计图；简单蓝色图使用基础检测，不能保证还原复杂组合及门序。照片、透视图、其他配色或完整软件截图可能漏检、误检；建议先裁出赛道区域。真实高度、通过方向、交叉及重复穿门仍需校对。连续导入时以最后一次为准，识别期间的工程编辑不会被旧结果覆盖。第一次使用可先看 [快速上手](docs/quickstart.md)，更多设置见 [使用说明](docs/usage.md)。

实拍 AI 功能使用固定官方端点 `https://api.deepseek.com` 和 `deepseek-flash` 模型，需要用户自己的 API 密钥；可选择保存到 Windows 凭据管理器。拖入多张照片会打开 AI 窗口，只有明确点击生成才发送缩放后的照片，生成后先预览，再决定是否应用。默认勾选“快速生成”，复杂组合可取消后重试；两种模式都可能漏识或返回无效结果，没有固定耗时保证。结果需人工校对，不属于测绘或摄影测量重建。

## VelociDrone 兼容状态

项目包含独立实现的原生 `.trk` 读写。**尚未完成安装版 VelociDrone 的实际导入和飞行验收。** 文件级往返测试不能替代模拟器验收；原生门架的尺寸换算也仍需实机校准。

测试范围和官方 AI 接口实测记录见 [验证记录](docs/validation.md)。

导出文件后，可依照 [VelociDrone 官方手册](https://www.velocidrone.com/desktop_manual)，将文件放入 Windows“文档”目录，再在模拟器的 Track Editor 中使用 Import Track。编辑器不直接改写模拟器数据库，也不要求模拟器账号凭据。对象支持范围和格式依据见 [格式说明](docs/format.md)。

## 构建与测试

当前目标平台为 Windows x64。需要 Visual Studio 2022 的 C++ 桌面开发工具、CMake 和 Qt 6.8.3 的 MSVC 2022 x64 组件。原生 `.trk` 加解密使用 Windows BCrypt；应用运行时不需要 Python。

在仓库目录运行，替换下面的 Qt 安装路径：

```powershell
cmake -S . -B build -DCMAKE_PREFIX_PATH="C:/Qt/6.8.3/msvc2022_64"
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

发布 Windows 程序时，需使用对应 Qt 版本的 `windeployqt` 收集动态库和插件，并附带第三方许可说明。构建过程和 CI 使用 Python 下载 Qt 属于开发工具链，应用功能由 C++ 实现。参见 [Qt Windows 部署文档](https://doc.qt.io/qt-6/windows-deployment.html)。

也可使用项目提供的打包脚本；输出目录必须为新目录或空目录：

```powershell
./scripts/package.ps1 -QtDir "C:/Qt/6.8.3/msvc2022_64" -BuildDir build -OutputDir dist/OpenTrackStudio-Windows
```

脚本会部署动态库、平台与图片插件、Windows Schannel TLS 插件、MSVC 可再分发运行库、使用文档和原始示例。Qt 的许可及第三方声明从官方 QtBase 6.8.3 源码归档中收集，下载内容会校验 SHA-256；已解压对应源码时可通过 `OPENTRACK_QTBASE_SOURCE` 环境变量复用。GitHub CI 在测试通过后生成便携 ZIP。

## 开源与贡献

应用源码使用 [MIT 许可证](LICENSE)。Qt 及其依赖保留各自许可证，见 [第三方说明](THIRD_PARTY_NOTICES.md)。项目不包含 VelociDrone 的模型、贴图、原程序代码或用户提供的参考截图，也不隶属于 VelociDrone。

欢迎提交可公开使用的赛道图、识别问题和模拟器导入结果。提交测试图片前，请确认有权公开它们；提供模拟器版本、场景和复现步骤有助于定位兼容问题。

## English

OpenTrack Studio 0.1.3 is a C++/Qt desktop editor for FPV race layouts. Drop one overhead plan onto the import box, 3D scene or overhead view to generate a local draft. Dark outlined obstacles with a magenta route support compound gates; simpler blue-symbol plans use a basic detector. One Undo restores the pre-import project. Review positions, headings, heights and order in the editable 3D view before exporting a native VelociDrone `.trk`. The editor displays X/Y ground coordinates and Z height. No application login is required.

Dropping multiple photographs opens the optional AI workflow, accepting up to 8 images. Images are sent to the official DeepSeek API only after an explicit Generate action; preview the draft before applying it. Fast generation is enabled by default, but neither mode guarantees accuracy or response time. Local diagram recognition remains offline.

New stock-beam exports support standard/start gates, turning pillars and double, triple, ladder, cube and stacked-cube assemblies. Each frame cell uses 3 m centreline spacing and 3.2 m beams with a 0.1 m square section. Select the passage column/level for compound gates. Actual loading and flying in the installed simulator remain unverified. See the [usage guide](docs/usage.md), [format notes](docs/format.md), [validation record](docs/validation.md) and [third-party notices](THIRD_PARTY_NOTICES.md). Application source is MIT licensed; Qt is dynamically linked under its applicable open-source licenses.
