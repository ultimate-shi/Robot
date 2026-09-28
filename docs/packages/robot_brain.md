<!-- 作用：说明 robot_brain 的现存决策核心与后续接入边界。 -->
<!-- 使用方法：修改 Qwen 决策、动作白名单或任务状态前阅读本文件。 -->
# robot_brain 功能包

## 职责与边界

`robot_brain` 当前保留 Qwen 文字决策、动作白名单、场景快照、目标解析和任务状态的 Python 核心代码。网页、HTTP 服务和 ROS Bridge 已清理，当前没有本包的 ROS launch 或可执行入口。模型输出只是动作提案，正式任务执行必须经过授权、目标核对及导航安全链。

本包归 `brain_owner` 管理。任务规划服务端属于 `robot_navigation`，视觉服务端属于 `robot_perception`；未来重新接入 ROS 时须复核跨包接口。

## 验证

```bash
python3 -m pytest -q src/robot_brain/test
git diff --check
```

重新设计软件入口时，需要明确 `/mission/navigation_goal` 的 Nav2 消费者，以及任务取消和控制租约的所有者。
