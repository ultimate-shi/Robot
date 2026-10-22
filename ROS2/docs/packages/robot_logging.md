<!-- 作用：说明 robot_logging 的 MCAP 录制、索引和历史查询职责；使用方法：调整日志策略或排查 /log/ 前阅读。 -->
# robot_logging 功能包

## 职责与入口

`robot_logging` 在整车运行时录制日常 MCAP 和异常片段，按配额清理旧会话，并将已完成的文件索引到 SQLite。`robot_main` 默认以 `start_logging:=true` 启动录制和索引；单独启动可用：

```bash
ros2 launch robot_logging logging.launch.py
```

`log_manager` 负责录制、异常触发和文件轮转，`log_indexer` 负责索引。`/logging/trigger_incident` 可手动触发异常片段；ROS ERROR/FATAL 日志也会触发。日常录制排除大体积数据类型，异常片段保留完整数据，具体规则见 `config/logging.yaml` 与 `config/mcap.yaml`。

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
