"""作用：验证控制器启动按实际状态重试；使用方法：pytest -q src/robot_control/test/test_controller_bootstrap.py。"""

from types import SimpleNamespace

from robot_control import controller_bootstrap


def test_lifecycle_steps_skip_already_active_controller():
    assert controller_bootstrap.next_step(None) == 'load'
    assert controller_bootstrap.next_step('unconfigured') == 'configure'
    assert controller_bootstrap.next_step('inactive') == 'activate'
    assert controller_bootstrap.next_step('active') == 'done'
    assert controller_bootstrap.next_step('finalized') == 'error'


def test_lost_load_response_recovers_from_actual_state(tmp_path, monkeypatch):
    config = tmp_path / 'controllers.yaml'
    config.write_text(
        'wheel_controller:\n  ros__parameters:\n'
        '    type: velocity_controllers/JointGroupVelocityController\n',
        encoding='utf-8')
    monkeypatch.setattr(controller_bootstrap.time, 'sleep', lambda _: None)

    class FakeBootstrap:
        timeout = 8.0

        def __init__(self):
            self.current = None
            self.calls = []

        def state(self, name):
            return self.current

        def set_parameters(self, name, path, plugin_type):
            self.calls.append('parameters')

        def call(self, service_type, suffix, request):
            self.calls.append(suffix)
            if suffix == 'load_controller':
                self.current = 'unconfigured'
                raise TimeoutError('应答丢失')
            if suffix == 'configure_controller':
                self.current = 'inactive'
            if suffix == 'switch_controller':
                self.current = 'active'
            return SimpleNamespace(ok=True)

        def get_logger(self):
            return SimpleNamespace(info=lambda _: None,
                                   warning=lambda _: None,
                                   error=lambda _: None)

    fake = FakeBootstrap()
    assert controller_bootstrap.ControllerBootstrap.ensure_active(
        fake, 'wheel_controller', str(config), 5)
    assert fake.calls == [
        'parameters', 'load_controller', 'configure_controller',
        'switch_controller']
