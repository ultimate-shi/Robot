<!-- 作用：说明 robot_web 的统一网页入口和人工控制台；使用方法：修改 9100 路由、页面或控制链前阅读。 -->
# robot_web 功能包

## 职责与入口

`robot_web` 将网页入口与控制台放在同一功能包。宿主机常驻的 `robot-web.service` 提供唯一的 9100 端口；整车启动时，`robot_main` 通过 `web_console.launch.py` 启动容器内的 ROS 控制台节点。两者通过工作区 `.runtime/robot-console.sock` 通信，不再开放第二个网页端口。

| 路径 | 用途 |
| --- | --- |
| `/robot/` | 手动驾驶、地图导航、相机和模式控制 |
| `/robot/status` | 机器人、导航、传感器与系统状态 |
| `/robot/console` | 终端式实时 ROS 输出 |
| `/log/` | `robot_logging` 的独立历史日志查询与 MCAP 下载 |
| `/v1/*` | 已有推理接口，由 9100 网关转发 |
| `/health` | 网页、日志与推理服务的健康状态 |

宿主机入口配置在 `config/web.yaml`，控制台 ROS 话题、手动命令超时、相机来源和可选模型接口在 `config/web_console.yaml`。静态页面位于 `web/`，ROS 节点和运动状态分别在 `robot_web/console.py` 与 `robot_web/control.py`。

## 控制安全

控制台无需网页心跳即可手动操作，且不设网页速度上限或独占控制权。多个页面同时操作时，最后到达的速度命令生效。松键和手动命令超时会停车。手动速度由控制台直接发布到 `/cmd_vel_safe`，绕过超声波避障；导航速度经 `/cmd_vel_console`、避障 `/cmd_vel_nav_safe` 后由控制台转发到 `/cmd_vel_safe`。底盘运动学、轮速保护和硬件故障保护仍生效。显式软件停止会锁存停车，直到页面点击“解除软件停止”；它还会取消网页任务与导航目标，并阻止受整车锁约束的定距 Action，且不替代物理急停。
首页“首页 · 机器人控制”标题下方单独显示底盘提示，包括未验证轮速被拒绝、故障锁定、控制板不可用、心跳异常和反馈中断；过期反馈会明确标为过期，不以旧的输出状态显示为正常。

固定在真实环境中的相机数据默认不用于运动中的虚拟机器人。探索、跟随、视觉选目标和模型任务依赖相应地图、定位、检测或推理服务；不可用时页面应显示状态。外部模型为可选配置，模型任务需经过白名单和用户确认。

## 启动与验证

```bash
colcon build --symlink-install --packages-select robot_web
source install/setup.bash
ros2 launch robot_web web_console.launch.py --show-args
pytest -q src/robot_web/test
```

完整运动链应通过 `robot_main` 启动。只启动本包控制台时，不会自行启动底盘和避障节点。
