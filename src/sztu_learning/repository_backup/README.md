# 仓库源码备份与恢复

2026-11-21 将学习代码、笔记、配置、最终二维地图与本轮最终 PCD 纳入主项目 Git 仓库。
此次上传不改变现有程序的目录和运行路径。

## 新项目源码完整备份

- `agibot-x2-navigation-full-9ad1ad8.tar.gz.partNNN`：原始完整离线归档按 32 MiB 分卷，包含上游 Git 历史及已下载的 LFS 实体，无需 GitHub 即可恢复。
- `archive_parts.json`：各分卷和合并归档的 SHA-256、大小。
- `agibot-x2-trial-source.tar.gz`：当前适配后的 `upstream/` 全部工作区文件，不包含嵌套 `.git`；可直接恢复并构建。
- `trial_source.json`：适配源码归档的 SHA-256、上游版本、文件数。
- `../agibot_x2_trial/evidence/upstream.patch`：适配修改的文本补丁；完整适配快照已另存，不依赖补丁恢复。

先在本目录重组原始归档并校验：

```bash
python3 reassemble.py
```

脚本默认生成 `restored/agibot-x2-navigation-full-9ad1ad8.tar.gz`，不覆盖已有文件。
确认目标目录不存在后，在学习目录执行：

```bash
tar -xzf repository_backup/restored/agibot-x2-navigation-full-9ad1ad8.tar.gz -C external_projects
tar -xzf repository_backup/agibot-x2-trial-source.tar.gz -C agibot_x2_trial
```

第二条恢复适配后的 `agibot_x2_trial/upstream/`；两个归档的源码与许可证均保留。
嵌套仓库在外层 Git 中通过归档备份，避免克隆主项目后只剩无法恢复的 gitlink。

## 上传范围

包括整个学习目录内未被忽略的源码、测试、学习笔记、参数、launch、RViz 配置、结果报告、轨迹 CSV、预览图与二维地图。
本轮最终 PCD 为 `agibot_x2_trial/maps/mapping02/map.pcd`，同时保存同源原始地图、地面变换与轨迹。
原项目其他尚未提交的开发修改保持原样，不混入这次学习目录备份提交。

原始传感器录包、逐帧点云分块、大型中间数组/射线数据、逐条事件日志、运行日志目录、编译与安装产物只保留本地，不是这次 Git 代码备份的一部分。
缺少录包时可以恢复源码和查看保存的地图，无法重新执行相应录包实验。
系统 ROS/PCL/Nav2 依赖需按试用说明重新准备；原始包与地图摘要见 `agibot_x2_trial/evidence/manifest.json`。

主入口：[试用验证记录](../agibot_x2_trial/试用验证记录.md)、[运行说明](../agibot_x2_trial/README.md)。
