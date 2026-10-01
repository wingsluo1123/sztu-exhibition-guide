<!-- GitHub Copilot 读这个文件。内容与仓库根的 CLAUDE.md / AGENTS.md 一致，
     那两个是符号链接指向同一份；这一份必须是真文件（Copilot 不跟软链）。
     **改动时三处要一起改** —— 实际只需改 CLAUDE.md，再把本文件同步过来。 -->

# 给接手这个项目的 AI 助手

完整版见仓库根的 `CLAUDE.md`。这里是**最容易判断错、错了代价最大**的一条。

## 厂商的 SLAM / 导航自主包不会开通。别去找它的接口怎么用。

**2026-10-01 已向厂商询价，费用过高，项目决定不采购。这是采购决策，不是待解决的技术问题。**

**不要**研究 `/agibot/software/nav/` 下那套怎么启动、怎么配、怎么绕过授权、
那几个模块为什么起不来。答案已知且不会变：**没买。**
自研是**唯一路线**，不是备选。

### 现场证据极具误导性

| 你会看到 | 容易得出的错误结论 | 事实 |
|---|---|---|
| `/agibot/software/nav/` 整套代码配置都在 | "配对就能跑" | 代码在，授权不在 |
| `map_manager` / `auto_integrated` **进程在跑** | "功能是活的" | **进程活着 ≠ 模块初始化成功** |
| `/pnc/*`、`/perception/grid_map` 有发布者 | "导航栈能用" | 这两个确实有授权，但编排层 `IntegratedControlModule` 被拒 |
| 日志写 `temporary cert expired` | "续期就行" | 文案误导 |
| `license.yaml` 里 `slam: authorized: true` | "明明授权了" | 那是过期的 3 天临时授权，只是回退源 |

最硬的佐证：`uwb_fusion`(700302) **在**正式授权里，`slam`(700301) **不在** ——
相邻组件 ID，一个买了一个没买，"过期"解释不了这个差别。

## 两套 "SLAM" 别混

```
/slam/mapping/odometry        厂商的，发布者永远 0（模块被拒）
/slam/localization/odometry   厂商的，发布者永远 0
/slam/lidar_odom              ← 我们的口子，lio_odom_bridge 发的
```

实际在用的建图链路：
`fastlio_mapping` → `/cloud_registered` → `tools/gridmap.py` → `maps/*.pgm`

## 缺口（按紧急程度）

1. **避障** —— `lidar_callback` 还是 `pass`，**展厅硬前置**
2. **回环** —— FAST-LIO2 是纯里程计，漂移沿路程单调累积
3. **重定位** —— 靠每趟从贴胶带的固定点起步顶；退路是 ArUco 地标

其余约定（测试怎么跑、净空两个阈值、domain 串台等）见 `CLAUDE.md`。
