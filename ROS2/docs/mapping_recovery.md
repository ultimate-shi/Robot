<!-- 作用：说明 RTAB-Map 异常退出后的恢复保存和内存治理；使用方法：Jazzy 容器加载 /workspace/install/setup.bash 后执行命令。 -->
# 地图异常恢复与内存排查

## 正常建图与异常恢复

`robot_main robot.launch.py mode:=real` 的活跃库默认是 `/workspace/mapping/rtabmap.db`，
位于宿主机 `ROS2/mapping/` 挂载目录，退出或重启容器不会自动删除。建图入口已移除
`--delete_db_on_start`；重新启动会打开原库，新建地图应显式使用另一个文件名：

```bash
ros2 launch robot_main robot.launch.py mode:=real mapping_database_path:=/workspace/mapping/session_02.db
```

`mapping_database_path` 只用于在线建图，已有地图定位仍使用 `map_yaml_file` 和 `database_path`。
建图与快照节点必须使用同一个活跃库。只运行独立 `rtabmap_mapping.launch.py` 时，参数名是
`database_path`，留空沿用所选建图 YAML；单独运行快照节点时也要指定同一路径。

默认 `mapping_recover_on_exit=true`：建图进程异常退出且 launch 仍在运行时，启动独立恢复进程。
正常关闭整车不触发自动恢复；整个容器被关闭/杀死后需手动执行恢复。恢复不自动重启在线建图或底盘。
网页“保存地图”对应的 `/mapping/save_snapshot` 在所有建图服务消失后自动回退，也可显式调用：

```bash
ros2 service call /mapping/recover_snapshot std_srvs/srv/Trigger '{}'
```

服务不存在或整个系统已退出时，只启动容器并加载工作区环境，然后运行：

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch robot_navigation mapping_recovery.launch.py

# 旧版本的故障库仍在 /tmp，需要显式指定；不要先启动旧版建图入口
ros2 launch robot_navigation mapping_recovery.launch.py database_path:=/tmp/robot_rtabmap_mapping.db

# 也可使用一次性命令，失败时返回非零退出码，便于脚本检查
ros2 run robot_navigation mapping_recover --database-path /workspace/mapping/rtabmap.db
```

自动恢复和命令都遵循同一流程：

1. 检查原库和 `.back` 是否仍被进程打开；仍在线、无法确认相关进程或有并发恢复时拒绝。
2. 对两份库分别执行 SQLite 完整性及非空节点检查，按最后观测时间、最大节点 ID、数量选择有效库。
   损坏的原库可以回退到有效备份；不会仅根据文件修改时间选库。
3. 确认磁盘能放下完整数据库副本及预留空间，使用 SQLite 备份接口复制到暂存目录。
4. 对副本运行 `rtabmap-export --map --opt 2 --save_in_db --threads 1`，优先使用已保存的优化地图，
   必要时从局部栅格重建；优化结果写回副本，保持地图和数据库一致。原库不被写入或替换。
5. 完成文件校验、SHA-256、属主和权限设置后，原子公开 `/workspace/maps/map_时间戳/`。
   失败或导出超时不会留下看似完整的地图目录。

恢复至少包含 `map.{yaml,pgm,json,db}` 和 `recovery.log`。`map.json` 记录来源、最后观测、
节点数、是否恢复、点云来源及校验摘要。缓存 PLY 只在来源数据库、二维地图像素、分辨率和原点
与本次导出相符时复用；缓存损坏或不匹配不会阻止二维地图和定位库恢复。
**无缓存时默认不重建三维点云**，会记录 `cloud_available=false`，这类包可用于实机地图定位，
不能充当完整三维仿真环境。需要重建时复制 `config/mapping_recovery.yaml`，设置
`export_cloud: true`，再用 `config_file:=配置绝对路径` 启动；三维导出按配置降采样，资源需求更高。
工具的 `--save_in_db` 不会另存 PLY，因此恢复器先写回二维优化结果，再独立导出三维并把
`map_cloud.ply` 规范为 `map.ply`。已使用本次 1.1 GB 故障库副本验证三维恢复，生成约 8.2 MiB PLY。

恢复导出默认上限 180 秒；大数据库的复制和校验另需时间，网页控制台/网关分别等待 300/330 秒。
这些值可按现场数据库大小调整。`minimum_free_bytes` 默认额外预留 256 MiB，不是三维导出体积保证。
数据库损坏且无有效备份、没有可导出栅格、磁盘已满时会明确失败；不要删除原始现场。
尚未落盘的 RAM 观测无法恢复。只备份 `/tmp` 也无法抵御整个容器删除，因此新默认库使用挂载目录。

## 恢复后的使用

加载恢复包进行定位：

```bash
ros2 launch robot_main robot.launch.py mode:=real map_yaml_file:=/workspace/maps/恢复目录/map.yaml
```

先在网页确认实车位置与朝向，并等待定位许可后再导航。要继续增量建图，先复制到新的活跃库，
避免修改已完成的地图包并破坏其摘要：

```bash
mkdir -p /workspace/mapping
cp /workspace/maps/恢复目录/map.db /workspace/mapping/resumed.db
ros2 launch robot_main robot.launch.py mode:=real mapping_database_path:=/workspace/mapping/resumed.db
```

该方式会继续写新观测；重启后里程计会重置，机器人也可能移动过，旧图与新观测仍需可靠匹配。
不能把“打开旧数据库”视为已经定位成功，也不自动恢复之前的驾驶任务。

## 2026-10-31 内存故障证据与改动

当天运行日志记录 RTAB-Map 于北京时间 15:41:33 以 -9 退出，容器 `memory.events` 的
`oom_kill=1` 与之吻合；没有内核日志读取权限，无法进一步取得该次 OOM 的逐进程 RSS。
故障备份的最后观测为 15:40:28，共 3051 个节点，图像和深度压缩数据合计约 681 MiB。
从数据库 Statistics 解压得到：

| 节点 ID | 工作内存节点数 | 短期节点数 | 视觉词典词数 | 数据库统计 MB |
| --- | ---: | ---: | ---: | ---: |
| 100 | 90 | 10 | 17976 | 35 |
| 1000 | 990 | 10 | 309779 | 366 |
| 2000 | 1990 | 10 | 633999 | 694 |
| 3051 | 3041 | 10 | 958320 | 1049 |

原配置静止也以 1 Hz 观察，`Mem/RehearsalSimilarity=1.0` 禁止合并，
`Rtabmap/MemoryThr` 未设置（默认 0，无限），工作节点与词典随时间累积。
这已能解释主要增长，无证据表明 Python 快照节点在积累全部历史消息。
上游 backup 实现还会关闭、复制、重新初始化数据库，可能在已有高占用时触发重载峰值；
不能仅凭退出时间断定其内部哪一步耗尽内存。

本次保留静止观察、动态射线清空和 0.02 m 全局地图，调整为：

- 相似度 0.6 合并、更新合并节点到最新 ID，工作内存目标 200 节点、短期缓存 10 节点，旧节点转长期库。
- `Mem/LoadVisualLocalFeaturesOnInit=false`、`map_cache_loaded_on_init=false` 按需重载；
  原始图像不保留在 RAM，数据库不放内存，SQLite 缓存设为 2000 页。
- 清理无用局部缓存、对近邻重复位姿过滤并体素合并输出点云；不以最近若干节点裁掉全局地图。
- 使用当前参数名 `Grid/Sensor=1`，启用 `/info` 中的 `Memory/RAM_usage/MB`，便于后续长时间追踪。

`MemoryThr` 是节点目标，不是 MB，也不是进程 RSS 硬限制：局部保护及重取会临时超出阈值，
大范围全局栅格、索引、分配器和其他进程仍占内存。更新合并 ID 会保留已写盘的历史节点
（weight=-9），数据库仍可能增长；磁盘历史增长不等于工作内存泄漏。
在真实场景长期运行后，应同时查看 `/info` 的工作节点、词典、RAM 统计和系统可用内存，
不能仅凭一个文件大小或短时回归宣称消除了全部 OOM。

参数语义与保存实现参考 [RTAB-Map 参数定义](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)、
[ROS backup 实现](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/src/CoreWrapper.cpp)；
实施时另核对了板端安装的 0.23 参数头文件。

## 验证

普通恢复回归覆盖损坏库回退、最新有效库选择、原库不变、运行库拒绝、互斥、空间不足、导出失败及超时不留半成品。
独立 DDS 域中的合成双目/RGB-D 集成测试用 SIGKILL 模拟无清理退出，分别验证保存服务回退和退出事件自动恢复，
并用恢复库重新定位及确认导航许可。合成内存回归分别测试静止观察合并、工作内存转移，
转入长期库的节点仍可在数据库中找到；测试不连接实车、不发送底盘运动目标。

本次 200 帧扩展验证：静止时工作/短期节点为 0/1；移动输入使用 20 节点目标，结束时为 29/5，
数据库仍保留 200 个节点，证实转出工作内存没有删除已建地图。该短时合成验证不替代实机场景长期 RSS 观察。

```bash
ROS_DOMAIN_ID=179 RUN_RTABMAP_INTEGRATION=1 python3 -m pytest -q src/robot_navigation/test/test_rtabmap_roundtrip.py
ROS_DOMAIN_ID=177 RUN_MAPPING_MEMORY_INTEGRATION=1 python3 -m pytest -q -s src/robot_navigation/test/test_mapping_memory_integration.py
```
