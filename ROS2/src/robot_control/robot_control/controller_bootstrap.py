#!/usr/bin/env python3
"""作用：限时、可重试地启动单个 ros2_control 控制器；使用方法：由 controllers.launch.py 顺序调用。"""

import argparse
import sys
import time

import rclpy
import yaml
from controller_manager_msgs.srv import (
    ConfigureController, ListControllers, LoadController, SwitchController)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node


def controller_type(config_file, name):
    """使用方法：从控制器参数文件读取插件类型，保留原有 YAML 入口。"""
    with open(config_file, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    return config[name]['ros__parameters']['type']


def next_step(state):
    """使用方法：根据真实生命周期状态决定下一步；重复执行不会重新配置 active 控制器。"""
    return {
        None: 'load',
        'unconfigured': 'configure',
        'inactive': 'activate',
        'active': 'done',
    }.get(state, 'error')


class ControllerBootstrap(Node):
    """控制器启动服务每次调用限时，响应丢失后按实际状态重试。"""

    def __init__(self, timeout):
        super().__init__('controller_bootstrap')
        self.timeout = timeout

    def call(self, service_type, suffix, request):
        """每次新建客户端，避免上次超时的 future 干扰重试。"""
        client = self.create_client(service_type, '/controller_manager/' + suffix)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise TimeoutError(f'{suffix} 服务不可用')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
            if not future.done():
                raise TimeoutError(f'{suffix} 响应超时')
            if future.exception() is not None:
                raise RuntimeError(f'{suffix} 调用失败: {future.exception()}')
            return future.result()
        finally:
            self.destroy_client(client)

    def state(self, name):
        response = self.call(ListControllers, 'list_controllers',
                             ListControllers.Request())
        return next((item.state for item in response.controller
                     if item.name == name), None)

    def set_parameters(self, name, config_file, plugin_type):
        parameters = []
        for key, value in (
                ('params_file', ParameterValue(
                    type=ParameterType.PARAMETER_STRING_ARRAY,
                    string_array_value=[config_file])),
                ('type', ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=plugin_type))):
            parameters.append(Parameter(name=f'{name}.{key}', value=value))
        response = self.call(SetParameters, 'set_parameters',
                             SetParameters.Request(parameters=parameters))
        if len(response.results) != len(parameters) or not all(
                item.successful for item in response.results):
            raise RuntimeError(f'{name} 参数设置失败: {response.results}')

    def ensure_active(self, name, config_file, max_attempts):
        plugin_type = controller_type(config_file, name)
        last_error = None
        for attempt in range(1, max_attempts + 1):
            try:
                state = self.state(name)
                step = next_step(state)
                if step == 'done':
                    self.get_logger().info(f'{name} 已激活')
                    return True
                if step == 'error':
                    raise RuntimeError(f'{name} 状态无法继续启动: {state}')
                if step == 'load':
                    self.set_parameters(name, config_file, plugin_type)
                    response = self.call(LoadController, 'load_controller',
                                         LoadController.Request(name=name))
                elif step == 'configure':
                    response = self.call(
                        ConfigureController, 'configure_controller',
                        ConfigureController.Request(name=name))
                else:
                    request = SwitchController.Request()
                    request.activate_controllers = [name]
                    request.strictness = SwitchController.Request.STRICT
                    request.timeout.sec = int(self.timeout)
                    request.timeout.nanosec = int(
                        (self.timeout - int(self.timeout)) * 1_000_000_000)
                    response = self.call(SwitchController, 'switch_controller', request)
                if not response.ok:
                    raise RuntimeError(f'{name} {step} 服务拒绝')
                # 切换可能完成但应答丢失；下一轮先读实际状态。
                last_error = None
            except (TimeoutError, RuntimeError) as exc:
                last_error = exc
                self.get_logger().warning(
                    f'{name} 启动第 {attempt}/{max_attempts} 次未完成: {exc}')
            time.sleep(0.2)
        try:
            if self.state(name) == 'active':
                return True
        except (TimeoutError, RuntimeError) as exc:
            last_error = exc
        self.get_logger().error(f'{name} 未能激活: {last_error}')
        return False


def main(args=None):
    parser = argparse.ArgumentParser(description='限时启动单个 ros2_control 控制器')
    parser.add_argument('controller')
    parser.add_argument('--param-file', required=True)
    parser.add_argument('--service-timeout', type=float, default=8.0)
    parser.add_argument('--max-attempts', type=int, default=12)
    options, _ = parser.parse_known_args(args if args is not None else sys.argv[1:])
    if options.service_timeout <= 0 or options.max_attempts < 1:
        parser.error('超时和重试次数必须为正数')
    rclpy.init(args=args)
    node = ControllerBootstrap(options.service_timeout)
    try:
        success = node.ensure_active(
            options.controller, options.param_file, options.max_attempts)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
