<!-- 作用：说明 robot_brain 的现存决策核心与后续接入边界。 -->
<!-- 使用方法：修改 Qwen 决策、动作白名单或任务状态前阅读本文件。 -->
# robot_brain 功能包

## 职责与边界

`robot_brain` 保留 Qwen 文字决策、动作白名单、场景快照、目标解析和任务状态的 Python 核心代码。网页、HTTP 服务和 ROS Bridge 不在本包；本包没有 ROS launch 或可执行入口。`robot_web` 控制台已调用本包的模型解析与动作策略，模型输出只是动作提案，网页确认后才由控制台提交相应导航任务。

任务规划服务端属于 `robot_navigation`，视觉服务端属于 `robot_perception`；调整控制台任务适配器时须复核跨包接口。

## 验证

```bash
python3 -m pytest -q src/robot_brain/test
git diff --check
```

`robot_brain` 内仍保留早期多用户租约状态实现；当前网页控制台没有独占控制权，任务确认、取消和软件停止由 `robot_web` 管理。
