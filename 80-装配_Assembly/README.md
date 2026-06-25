# UAM · 通用装配建模 (Universal Assembly Modeling)

让 AI 像人搭积木一样,把一堆零件**装**成能动的复杂装配体——
不是把网格摆到写死的坐标旁,而是 **感知配合特征 → 声明配合关系 → 求解位姿 → 验证闭环**,
让正确的装配自己解出来。第一个实践锚点:**SR6**(6 自由度并联机构)。

道法自然,无为而无不为:`损之又损`——把"逐件手调坐标"不断减掉,直到只剩"声明关系、让它自解"。

## 为什么需要它
见 [`docs/ROOT_CAUSE.md`](docs/ROOT_CAUSE.md):之前 SR6 "怎么都装不起来",根因不在 IK/渲染/能力,
而在**"装配"这一层从未真正存在**——零件是按 magic offset"贴"到正确骨架旁的,彼此之间没有真实配合约束。

## 架构(五层,生成—验证闭环,源自预测编码)
见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。
`L0 感知 → L1 语义 → L2 配合 → L3 求解 → L4 运动学 → L5 验证`。
L0–L3 不含任何 SR6 专有逻辑;换装配体 = 换 `anchors/`,框架不动。

## 目录
```
uam/            通用内核(无 SR6 专有逻辑)
  perceive.py   L0 感知:原始网格 → 孔/轴/面/对称(低模稳健)  ✅ 已验证
anchors/sr6/    SR6 锚点
  constants.py  L4 固件 IK + 骨架常数(mesh 实测确认)        ✅ 就绪
ground_truth/   只读地面真值:firmware / pdf / stl / ref / prior
results/        感知/求解产物(perceive.json 等)
docs/           ROOT_CAUSE.md / ARCHITECTURE.md
```

## 已验证的硬事实
| 来源 | 事实 |
|---|---|
| 固件 IK | `set_main_servo` 中位 = 0 us,完整还原 |
| L0 感知反推骨架 | Arm=50 / Pitcher=75 / MainLink=175 / Receiver 偏置=55,全部从网格实测 |
| 变体陷阱 | `BearingMainLink`=135 ≠ `MainLink_Alpha`=175,混用必然闭环失败 |

## 进度
L0 感知 ✅ · L4 骨架 ✅ · L1 语义 ⏳ · L2 配合 ⏳ · L3 求解 ⏳ · L5 验证 ⏳
