# VelociDrone 互操作格式与验证边界

OpenTrack Studio 的适配器是独立编写的 C++ 实现。它在本地生成或读取 `.trk`，不连接账号服务、不读取密码、不改写模拟器数据库。Windows 版本使用系统自带的 CNG / BCrypt AES 实现。

## 原理

官方手册说明，VelociDrone 可以导出自定义赛道，也可以从 Windows 的 Documents 目录导入赛道。标准操作路径为模拟器的 Track Editor → Import Track。参考：[官方桌面手册](https://www.velocidrone.com/desktop_manual)。

公开的独立工具表明，`.trk` 的文件内容是 Base64 编码的 AES-128-ECB 密文，填充为 PKCS#7。这里使用的是文件互操作常量，并非用户账号凭据。解密后的 UTF-8 内容依次为：

```text
场景数字编号
赛道名称
单行 JSON
0
0
```

部分外部工具省略最后两行；本程序读取两种形式，输出带这两行的形式。名称中的换行会转换为空格，避免破坏文件结构。

JSON 的核心字段为 `gates` 与 `barriers`。门的一个最小例子如下，测试中的数据完全为本项目自行构造：

```json
{
  "gates": [{
    "prefab": 285,
    "trans": {
      "pos": [0, 0, 0],
      "rot": [1000, 0, 0, 0],
      "scale": [100, 100, 100]
    },
    "gate": 0,
    "start": true,
    "finish": true
  }],
  "barriers": []
}
```

`pos` 使用整数厘米，Y 向上；`rot` 是按 W、X、Y、Z 排列、放大 1000 倍的四元数；`scale` 为百分比。`gate` 从零开始表示通过顺序。编辑器平面坐标对应模拟器的 X/Z，编辑器的高度指门框中心高度。偏航 0° 朝 +Z，90° 朝 +X。门的原生原点在底部，所以输出普通门时会从中心高度减去门高的一半。

已核对的场景包括 Football Stadium = 8，Empty Scene Day = 16。导入的其他场景会显示为 `Scene 数字编号`，并保存编号。

## 当前支持

- 新建的标准门与起终点门导出为 prefab 285（DefaultMGPRF）。宽、高按标称 2.4 m 孔径计算缩放；这是待实机校准的尺寸基准，不代表已逐个核对游戏模型的实际孔径。
- 新建转向物为实体转向柱 `pillar`，不是旗帜。物理柱由原生 prefab 2218（BlockBlack）的 1 m 单位立方体拉伸而成，写入 `barriers`。例如宽 0.4 m、高 3 m 的柱使用 `scale: [40,300,40]`，柱体原点在底部，中心高度 1.5 m 时 `pos.y = 0`。渲染为同尺寸黑色实体柱，旗帜不会替代柱体。
- 柱旁另放一个 prefab 88（DefaultSquare）不可见顺序检查点，写入 `gates`，从而把物理障碍和通过顺序分开。按公开辅助几何边界，88 位于本地 Y/Z 平面、中心偏移为 `[0,0,-1]`；以 `[100,柱高×50,120]` 缩放，名义窗口宽 2.4 m、高度与实体柱一致，中心高度也与柱体一致。例如 3 m 柱使用 `[100,150,120]`。旋转为编辑器偏航减 90°，再补偿 `[0,0,-1.2]` 的原点偏移，使名义窗口法线与编辑器行进方向一致。
- 默认通过柱的本地右侧，也可由工程 `Gate.extra.passSide = -1` 选择左侧。检查窗口中心与实体柱中心的平面距离为 `柱宽 / 2 + 1.55 m`；其中 1.2 m 为半窗口宽、0.35 m 为净间隔。例如 0.4 m 柱的检查窗口中心在侧面 1.75 m 处，路线经过窗口中心，不穿过实体柱。这是本程序的明确编辑约定，不是对 DIA 官方绕柱规则的声称；一个相邻检查窗口也不能单独验证完整绕柱动作。
- 导入本程序的柱体输出时，按固定窗口宽度、柱高、位置、偏航及侧面间隔配对 2218 实体与 88 检查点，恢复为逻辑转向柱。配对与障碍物归属仅记录于 `.otproj` 的 `extra`，不向原生 `.trk` 注入自定义字段。重新导出会替换所属柱体，避免重复；删除、复制柱体也会同步删除、增加其物理方块。未配对的原生障碍物保持保留。
- 导入原有 prefab 170 旗帜仍保留其原生数据，仅用于旧赛道兼容。新建旗帜不再作为转向柱导出。
- 新建的双门、梯形组合、多门框立方组合（`cube`）和俯冲门暂不导出。程序明确报错，不把它们伪装成标准门。它们可编辑和保存为本程序工程；这不影响上述实体立方体转向柱的导出。
- 导入原生赛道时保留原始组件对象、四元数、缩放、未知字段、障碍物和天气。未修改的测试赛道再次导出可以与独立生成的密文逐字节一致。
- 导入的未知 prefab 在画布上采用简化图标。其原生数据仍保留；简化显示不能用来证明模型外观、碰撞体和通过方向完全一致。旗帜、俯冲门或不规则模型的中心位置和尺寸也仅作示意。
- 修改导入组件的位置、偏航和尺寸时，适配器尽量保留其原始倾斜姿态与其他字段；重新编号门序。一个赛道只能有一个起终点；没有指定时使用首个普通组件。只有转向柱的工程需要先补充起终点门。
- 赛道名、门位、门序等是可生成的数据。PNG/JPG 底图本身不会变成模拟器地形或贴图。

## 测试与尚未验证的部分

`tests/velocidrone_test.cpp` 使用独立的 .NET AES 实现生成的已知答案密文，测试中文名称、厘米坐标、四元数顺序、中心高度换算、原生数据保留、尺寸修改、实体方块位于 `barriers`、旁侧 88 检查点的原点和名义间隔、左右通过侧、柱体配对与删除复制、旧旗帜兼容、无效文件拒绝和失败时不覆盖原文件。

这验证的是本地编解码与数据保留。当前环境尚未在实际安装的 VelociDrone 中加载该输出，也未验证不同模拟器版本、场景地面高度或每种模型的孔径。特别是 88 的方向、原点和窗口尺寸来自辅助几何推导，尚未校准游戏内实际触发体与判定方向；旁侧检查点的导出属于实验性适配。0.35 m 是名义几何净间隔，不是已验证的游戏碰撞间隔。用户完成模拟器导入并核对实体柱、检查窗口和通过顺序后，才能确认具体版本、场景下的实际兼容性；本程序不声称符合完整官方联赛判定。

## 证据与实现来源

- [VelociDrone 官方手册](https://www.velocidrone.com/desktop_manual)：原生导入导出流程。
- [FPVTracksideCore 的 VDFileManager.cs](https://github.com/uewepuep/FPVTracksideCore/blob/master/ExternalData/VDFileManager.cs)：公开的独立 `.trk` 编解码、最小 JSON 结构与 prefab 对应；其[许可证为 MIT](https://github.com/uewepuep/FPVTracksideCore/blob/master/LICENSE.md)。该工具有自己的坐标缩放约定，不能将其 `TransScale = 200` 直接当作模拟器的米制单位。
- [FPV Track Viewer 的 trk.js](https://github.com/skermiebroTech/fpv-track-viewer/blob/main/trk.js)、[坐标说明 app.js](https://github.com/skermiebroTech/fpv-track-viewer/blob/main/app.js)、[prefab / 场景目录](https://github.com/skermiebroTech/fpv-track-viewer/blob/main/tracks/prefabs.json)：用于交叉核对格式事实。该仓库当前未提供开源许可证；本项目未复制其代码、模型、图标或赛道示例。
- [辅助几何边界目录](https://github.com/skermiebroTech/fpv-track-viewer/blob/main/tracks/prefab-dims.json)：88 的本地边界记录为 `[-0.04,-1,-2,0.04,1,0]`。这支持检查平面的几何推导，不能替代游戏内实际触发体测试。2218 的单位方块与底部原点由上述 `app.js` 的独立场景实现交叉核对。
- [VeloTrackEditToolbox](https://github.com/lephro/VeloTrackEditToolbox)：可佐证赛道数据也存放于本地 SQLite 数据库。该仓库当前也未提供开源许可证，本项目未复制其实现。

本项目不分发模拟器原有的模型、贴图和赛道，不声称是 VelociDrone 官方产品。
