<!-- 作用：说明 robot_logging 的 MCAP 录制、索引和历史查询职责；使用方法：调整日志策略或排查 /log/ 前阅读。 -->
# robot_logging 功能包

## 职责与入口

`robot_logging` 在整车运行时录制日常 MCAP 和异常片段，按配额清理旧会话，并将已完成的文件索引到 SQLite。`robot_main` 默认以 `start_logging:=true` 启动录制和索引；单独启动可用：

```bash
ros2 launch robot_logging logging.launch.py
```

`log_manager` 管理日常/异常 MCAP 录制与配额轮转；`log_indexer` 独立筛选索引，原始 MCAP 不因索引采样、过期或配额淘汰而被改写。日志总配额和 7 天录制保留期仍由管理器执行，活动录制不会删除，启动下一片前会预留空间。

话题分别维护在两个文件中，均按 **常用、按需、不建议** 分类，每项显式设置 `enabled: true/false`。分类是维护建议；是否启用只由开关决定，未列出或未启用的话题默认不进入对应流程。修改后需重启相应日志进程。

| 文件 | 作用与默认行为 |
| --- | --- |
| `config/recording_topics.yaml` | 日常录制白名单：保留 ROS 输出、TF、融合/轮侧/视觉里程计、最终 IMU、关节反馈、完整速度链、定位/导航/任务事件、硬件诊断及八路超声波；重复 IMU 中间数据、串口流水、心跳、图像、点云和地图原始数据默认关闭。 |
| `config/index_topics.yaml` | 历史查询独立白名单：记录告警与关键节点 INFO、定位/硬件/安全/任务状态变化和动作状态，导航反馈最多每秒一条；TF、里程计、IMU、扫描数组等日常不逐条进入索引。 |
| `config/logging.yaml` | 根目录、总容量、磁盘余量、保留期、分片和索引配额；上述两个文件路径相对本文件解析。 |
| `config/mcap.yaml` | MCAP 分块压缩。 |

例如需要临时排查串口协议，将录制文件“按需”中的 `/hardware/serial_tx` 改为 `enabled: true`；它不会连带进入索引，索引开关仍须单独设置。`topic` 表示完整话题名，`regex` 表示完整名称匹配的正则表达式；两者只能选择一个。所有关闭项不构成强制黑名单，开启宽泛正则时需注意覆盖范围。空白名单会拒绝启动，避免意外变成全量录制。

索引 `mode` 可选 `all`（全部）、`changes`（变化）、`sample`（采样）、`rosout`（日志等级与重复合并）、`diagnostics`（异常诊断与恢复）。`fields` 只决定比较哪些状态字段，保存的事件正文仍包含当时完整状态；`heartbeat_seconds` 决定重复状态的保留间隔。采样/变化判断不修改 MCAP。规则改变后会按新规则重新处理仍在保留期内的录制。

默认日志索引保留 WARN/ERROR/FATAL 和关键导航节点 INFO，同节点相同日志五秒内合并；定位、硬件、避障状态仅变化时记录，稳定状态每分钟保留一次；动作状态变化不受反馈采样限制。诊断的健康心跳不建立索引，但异常及其恢复事件保留。消息正文上限 2048 字节；只解码入选且不超过 64 KiB 的消息，大原始数据直接留在 MCAP，不再创建占位索引行。

`/web_console/navigation_status` 由网页控制台在目标、阶段、恢复次数或结果发生变化时发布，保留网页目标坐标、取消结果及本次失败原因，两个白名单默认启用。索引器可处理活动会话中已经关闭的分片，最后一个仍在写入的分片会跳过，避免必须等待整轮 15 分钟才看到历史。

索引默认上限 400 MB、保留 7 天；超限时滚动淘汰最旧查询行并回收空间，不再停止新增索引，不会因此删除 MCAP。MCAP 仍受日志总配额/保留期管理；已淘汰的旧索引行不会在下一轮扫描时反复重建。

`/logging/trigger_incident` 可手动触发异常录制；ROS ERROR/FATAL 也会触发。默认异常录制保留 10 秒白名单数据（独立 2 GB 上限和冷却时间），避免故障时临时订阅全部原始图像增加负载；原始图像专项排查时可将录制文件顶层 `incident_record_all` 改为 `true`，恢复日常使用后设回 `false`。

## 网页与数据边界

本包实现只读的历史日志查询页面和 API，由 `robot_web` 挂载在 9100 端口的 `/log/`。页面支持按时间、等级、节点、话题、服务、Action 和关键词查询，也可下载 MCAP。`/robot/console` 是 `robot_web` 的实时 ROS 输出页，两者用途不同。

修改日志根目录时，核对容器挂载、磁盘配额与 `robot_web/config/web.yaml` 中的日志配置路径。运行生成的 MCAP、索引数据库和快照不提交到仓库。

## 验证

```bash
colcon build --symlink-install --packages-select robot_logging
source install/setup.bash
pytest -q src/robot_logging/test
ros2 launch robot_logging logging.launch.py --show-args
```
