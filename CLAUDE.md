# 给接手这个项目的人（含 AI 助手）

这个文件只放**最容易判断错、而且错了代价最大**的几条。细节在 `ROADMAP.md`。

---

## 1. 厂商的 SLAM / 导航自主包不会开通。别再去找它怎么用。

**2026-10-01 已向厂商询价，费用过高，项目决定不采购。这是采购决策，不是待解决的技术问题。**

所以：**不要**再去研究 `/agibot/software/nav/` 下那套怎么启动、怎么配、
怎么绕过授权、那几个模块为什么起不来。答案已知且不会变：**没买。**

自研是**唯一路线**，不是备选，也不是"等授权下来就换掉"的临时方案。

### 现场证据极具误导性，下面每一条都会把人带沟里

| 你会看到 | 容易得出的错误结论 | 事实 |
|---|---|---|
| `/agibot/software/nav/` 整套代码和配置都在 | "配对就能跑" | 代码在，授权不在 |
| `map_manager` / `auto_integrated` **进程在跑**（各 2 个） | "功能是活的" | **进程活着 ≠ 模块初始化成功**，模块被拒了 |
| `/pnc/*`、`/perception/grid_map` 有发布者 | "导航栈能用" | pnc/perception 确实有授权，但编排层 `IntegratedControlModule` 被拒 |
| 日志写 `temporary cert expired` | "续期就行" | 文案误导。见下 |
| `/agibot/data/license/license.yaml` 里 `slam: authorized: true` | "明明授权了" | 那是 2026-08-01 的 **3 天临时授权**，早过期，且只是回退源 |

**判定这件事的最硬一条佐证**：`uwb_fusion`(700302) **在**正式授权里，
`slam`(700301) **不在** —— 相邻的组件 ID，一个买了一个没买。
"过期"解释不了这个差别。（另一条：`pnc` 和 `slam` 在那份过期 yaml 里
`expire_at` 完全相同，而 pnc 跑得好好的。）

被拒的五个模块：`MappingModule` / `LocalizationModule` /
`RelocalizationModule` / `MapManagerModule` / `IntegratedControlModule`。

详见 `ROADMAP.md` 的 **B4**。

---

## 2. "SLAM"在这个项目里指两套东西，别混

| | 厂商那套 | **我们这套（在用的）** |
|---|---|---|
| 位置 | `/agibot/software/nav/` | `src/fast_lio`（本仓库编译）+ `src/sztu_slam` |
| 状态 | 授权被拒，**一张图都没出过** | 在用，`~/sztu_lio_runs/` 下 20+ 趟录制 |
| 产物 | `/agibot/data/var/MapManagerModule/`（**目录不存在**） | `maps/testroom.{pgm,yaml}` |

话题名很像，特别容易认错：

```
/slam/mapping/odometry        厂商的，发布者永远 0（模块被拒）
/slam/localization/odometry   厂商的，发布者永远 0
/slam/lidar_odom              ← 我们的口子，lio_odom_bridge 发的
```

我们干的事就是：**自己算位姿，从厂商导航的输入口灌进去。**

---

## 3. 我们有什么、缺什么

| 功能 | 状态 |
|---|---|
| 建图 | ✅ FAST-LIO2。两趟独立录制重复性 **p90 = 0.10 m**（2026-10-01 实测） |
| 实时定位 | ✅ `lio_odom_bridge` → `/slam/lidar_odom` |
| 点到点导航 | ✅ `sztu_navigation`（控制律 + 到点判定 + 超时 + 卡住检测） |
| 选点 | ✅ `tools/pick_waypoints.py`（厂商也不管这个，`NaviPoint` 只有坐标） |
| 讲解编排与内容 | ✅ `sztu_guide` / `sztu_interaction` / `config/content_*.yaml` |
| **避障** | ❌ `lidar_callback` 还是 `pass`。**展厅硬前置** |
| **回环** | ❌ FAST-LIO2 是纯里程计，漂移沿路程单调累积、永不纠正 |
| **重定位** | ❌ 靠「每趟从贴胶带的固定点起步」顶。退路是 ArUco 地标 |

**因为没有避障，现在在用保守参数换安全**：`max_linear_speed` 从 0.5
压到 0.25（理由见 `config/waypoints.yaml` 的注释）。这笔代价只有等自研
避障做出来才能收回。

---

## 4. 几条反复踩过的坑

- **FAST-LIO 回放不是确定性的。** 同一个包跑两遍，轨迹中位差 0.013 m、
  p95 0.133 m、最大 0.371 m（ikd-Tree 的后台重建线程）。所以
  **「包还在，图随时能重出一张」是假的** —— 图必须和点位同一次提交进版本库。
- **图和点位必须同源。** 两趟录制 = 两个 FAST-LIO 原点 + 两个重力系 =
  两个坐标系。栽过一次，现在有门禁（`test_map_matches_waypoints.py`、
  `test_map_lineage.py`）。
- **两个净空阈值别混**：站得住 0.95 m（半径 0.40 + 到点容差 0.30 + 滑行 0.25），
  过得去 0.40 m。
- **「直线畅通」不等于「走得过去」。** 控制律 advance 段边走边拧，走的是弧线。
  见 `test_route_survives_the_control_law.py`。
- **开发机和真机共用 ROS domain 0**，同网段会串台。仿真固定跑 42。
- **实走前必须 `systemctl --user stop sztu-bridge`** —— 它自带一个
  `navigation_manager`（systemd `Restart=always`，每次开机都会自己起来），
  不停就是两路速度打架。
- **跑测试用 `tools/run_tests.sh`**，别直接 `pytest src/`。
