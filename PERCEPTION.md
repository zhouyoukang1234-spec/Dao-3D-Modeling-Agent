# 三维感知层 · Structural 3-D Perception（percept.*）

> 不出於戶，以知天下。AI 之「看」不是像素，而是结构。
> 本文总结当前系统的 3D 感知机制、缺陷，业界调研结论，以及新感知层的设计。

## 一、当前系统的感知机制（现状总结）

在 percept.* 之前，Agent 感知建模成果的通道有三条：

| 通道 | 工具 | 感知内容 | 局限 |
|------|------|----------|------|
| 标量通道 | `solid.measure` / `measure.*` / `analyze.*` | 体积、面积、包围盒、质心、单个子元素的长度/半径 | 只有数字，没有**结构**：不知道"哪个面是什么、面与面如何相接" |
| 光学通道 | `view.*`（matplotlib 多视角渲染 PNG） | 着色多视图图片 | 对 AI 价值极低：像素不可精确读取、不稳定、不可 diff |
| 文件通道 | `doc.inspect` / `doc.summarize` / `doc.diff` | .FCStd 的参数树（对象、属性、链接） | 是**构建历史**的感知，不是**几何结果**的感知；布尔运算后的真实形状读不到 |

**核心缺陷（此前的"盲人摸象"状态）：**

1. **无拓扑感知** — 不知道一个 shape 有哪些面、每个面是平面还是圆柱面、面与面如何相邻。
2. **无凹凸感知** — 无法区分外棱边（凸）与坑底边（凹），也就无法理解"孔、槽、凸台"。
3. **无特征语义** — 看到 7 个面，但不知道其中是"一块板上有一个直径 8 的通孔"。
4. **无空间关系** — 多个物体之间是接触、相交、包含还是分离，方向如何，全靠猜。
5. **无截面阅读** — 无法像 CT 一样切开实体读内部结构。
6. **不可描述** — 没有一条从几何到稳定、精确、可复述文本的通路（编程之于 AI 的可读性）。

## 二、业界调研结论（开源/研究前沿）

- **B-rep 图表示**（UV-Net / BRepNet / BRepGAT，Autodesk Research；ABC/Fusion360 Gallery 数据集）：
  把 B-rep 编码为**面-边邻接图**，节点带曲面类型/参数，边带曲线类型与**凹凸性 (convexity)**。
  这是机器理解 CAD 的事实标准表示 → 本层 `percept.topology` 直接产出该图（JSON，无需神经网络）。
- **特征识别**（FeatureScript、AAG-based feature recognition、MFCAD/HeteroMF）：
  从邻接图 + 凹凸性识别孔/槽/凸台等**制造特征**，是几何到工程语义的桥 → `percept.features`。
- **CAD-as-code**（CadQuery / build123d / OpenSCAD / Text2CAD / CAD-Coder 等 LLM-CAD 工作）：
  结论一致：AI 最高效的 CAD 界面是**精确文本/程序**，不是图像 → 本系统的 doc.synthesize
  已覆盖"写"，percept.* 补齐"读"，读写闭环。
- **空间场景图**（3D Scene Graph 系列）：对象间关系谓词（接触/包含/方位）构成场景级理解
  → `percept.relations` / `percept.scene`。
- **切片阅读**（CAM/3D 打印 slicer、医学 CT）：平面截面是无损读取内部结构的经典手段
  → `percept.section`。
- **GUI 截图/渲染识别路线**：对 LLM 精度低、不稳定、不可比对，仅适合人工复核 → 保留 view.* 作辅助，不作主通道。

## 三、新感知层设计（percept.* 六法）

设计三原则（对标编程语言的可读性）：**精确**（数值直读，无估计）、**稳定**（同一形状同一输出，可 diff）、**可描述**（可直接进入推理上下文的 JSON/文本）。

| 工具 | 感知维度 | 输出 |
|------|----------|------|
| `percept.topology` | 结构 | 面-边邻接图：面（类型/面积/质心/法向/轴/半径）、边（类型/长度/**凹凸性**）、邻接关系 |
| `percept.features` | 工程语义 | 通孔/盲孔/凸台/圆角，含半径、轴向、位置 |
| `percept.section` | 内部结构 | 任意平面截面 → 闭合环折线（精确坐标），CT 式阅读 |
| `percept.relations` | 空间关系 | 两两谓词：apart/contact/overlap/contains + 距离 + 方位（±x/±y/±z）+ 重叠体积 |
| `percept.scene` | 全局 | 整文档摘要：所有对象 bbox/体积/面数 + 关系矩阵 |
| `percept.describe` | 语言 | 一形一段稳定文本：尺寸、拓扑构成、凹凸统计、特征清单、质心 |

**凹凸性判定**（`_edge_convexity`）：在棱边中点取两邻面的外法向 n1,n2 与面内方向 d1,d2，
`d2·n1 + d1·n2 < 0` 为凸（盒棱），`> 0` 为凹（坑底边），法向近平行为 smooth（圆角过渡）。
纯几何判定，无采样误差。

**孔识别**：凹圆柱面（法向指向轴线）→ 孔；沿轴跨满包围盒 → 通孔，否则盲孔。凸圆柱面按半径
相对尺度分派 boss / fillet。

## 四、感知-行动闭环（升级后）

```
写:  doc.synthesize / solid.* / param.*      （构建）
读:  percept.topology → 结构    percept.features → 语义
     percept.section  → 内部    percept.relations → 空间
     percept.describe → 语言    solid.measure     → 标量
验:  与意图比对 → 不符则改 → 再读 → 收敛
```

Agent 现在可以在**从零搭建**或**接手既有模型**时先 `percept.scene` 全局扫描，再对目标对象
`percept.describe` + `percept.topology` 建立精确心智模型，随后每一步操作后用结构 diff 验证
效果 —— 看得见，理解清楚，然后行动。

## 五、后续演化方向

1. **percept.diff**：两个 shape 的结构级比对（新增/消失的面与特征）— 操作验证的最强形式。
2. **特征库扩展**：槽（slot）、台阶（step）、倒角（chamfer）、阵列检测（重复特征分组）。
3. **对接成熟体系**：导出 STEP + occwl/UV-Net 图特征，为未来神经几何理解预留接口。
4. **装配语义**：在 relations 之上识别配合关系（同轴、面贴合、间隙配合）。

## 六、实践进化日志（感知→行动→验证→进化）

每一轮实战都必须暴露真实缺陷并当场修复，改进随即以回归测试固化：

| 轮次 | 战役 | 暴露的缺陷 | 落地的进化 |
|---|---|---|---|
| 1-3 | 法兰联轴器装配 + FEM + 工程图（69/69） | 盲/通孔误判、凸台读作圆角、无阵列、无 diff、干涉只能两两比 | 轴端探针、切向平滑判据、圆周/线性阵列识别、percept.diff、全装配干涉扫描 |
| 4 | 跨模块（PartDesign/齿轮/CAM/网格，25/25） | —（上轮修复后零缺陷） | 阵列感知直接驱动 CAM 选孔 |
| 5 | 单级减速箱（壳体+阶梯轴+运动学，29/29） | 环形面凹凸性误判（质心在孔内）、shell 需手查面索引、github 资源搜索崩溃 | 棱边切向×法向探针定面内方向、shell 支持 'zmax' 语义选择器、GitHubClient.search |
| 6 | 管路撬块（wire/bop/bim/measure，21/21） | wire.extrude 不认 out/dir、bim 按 Label 找不到对象、solid.* 看不见其他模块产物 | 参数别名归一、Label 解析、跨模块实体自动收养 |
| 7 | 涡轮叶片自由曲面（surface/mesh/points/ss，13/13） | mesh.analyze/export 无法读取 mesh.from_shape 产出的保留网格 | 统一走 _resolve_mesh：保留网格与实体细分同路解析 |
| 8 | 机加工支架（draft/fem/path/doc 版本，19/19） | 倒角后 percept.describe 在缝合棱边上崩溃（OCC undefined curve type） | _curve_info 容忍无法求曲线类型的棱边（报 unknown 而非崩溃） |

感知层本身也在被感知：每轮实战先用 percept.* 读出真实结构，与设计意图比对，
不符即为缺陷 —— 缺陷是进化的入口，循环不息。

道法自然，无为而无以为。
