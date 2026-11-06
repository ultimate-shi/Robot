"""作用：在 SLAM 退出后从残留数据库原子恢复地图；用法：ros2 run robot_navigation mapping_recover --help。"""

import argparse
from contextlib import closing
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import tempfile
import time
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import yaml

from robot_navigation.mapping.database import copy_database, file_digest, validate_bundle, validate_database


def load_options(filename):
    """CLI、退出事件及快照服务使用同一份恢复配置。"""
    with open(filename, encoding='utf-8') as stream:
        return yaml.safe_load(stream)['mapping_recovery']['ros__parameters']


def assert_databases_closed(paths):
    """离线恢复必须在数据库无人使用时进行，防止把运行中的库当成退出现场。"""
    identities = {(p.stat().st_dev, p.stat().st_ino) for p in paths if p.is_file()}
    for process in Path('/proc').glob('[0-9]*'):
        if process.name == str(os.getpid()):
            continue
        try:
            descriptors = list((process / 'fd').iterdir())
        except FileNotFoundError:
            continue
        except PermissionError:
            # 不要求查看无关用户的进程；无法查看 RTAB-Map 时保守拒绝。
            try:
                if 'rtabmap' in (process / 'comm').read_text():
                    raise ValueError('无法确认 RTAB-Map 是否已退出，请用运行建图的用户恢复')
            except FileNotFoundError:
                pass
            continue
        for descriptor in descriptors:
            try:
                info = descriptor.stat()
            except (FileNotFoundError, PermissionError):
                continue
            if (info.st_dev, info.st_ino) in identities:
                raise ValueError(f'数据库仍被进程 {process.name} 使用，请等待建图完全退出后再恢复')


def database_details(path):
    """按最后观测及节点数比较有效库，不以复制文件的修改时间冒充数据新鲜度。"""
    validate_database(path)
    with closing(sqlite3.connect(Path(path).resolve().as_uri() + '?mode=ro', uri=True)) as connection:
        count, last_id, stamp = connection.execute(
            'SELECT COUNT(*), MAX(id), MAX(stamp) FROM Node WHERE id > 0').fetchone()
    return {'node_count': count, 'last_node_id': last_id, 'last_node_stamp': float(stamp or 0)}


def attach_matching_cloud(directory, preview_directory, database_path):
    """仅地图像素、分辨率、原点完全配套时使用缓存点云；否则明确留下二维恢复包。"""
    preview = Path(preview_directory)
    if not all((preview / ('current' + ext)).is_file() for ext in ('.yaml', '.pgm', '.ply', '.json')):
        return False
    manifest = json.loads((preview / 'current.json').read_text())
    if manifest.get('database_path') != str(Path(database_path).expanduser().resolve()):
        return False
    saved = yaml.safe_load((directory / 'map.yaml').read_text())
    cached = yaml.safe_load((preview / 'current.yaml').read_text())
    if (not math.isclose(float(saved['resolution']), float(cached['resolution']), rel_tol=1e-5)
            or not np.allclose(saved['origin'], cached['origin'], atol=float(saved['resolution']) * 1e-3, rtol=0)):
        return False
    first = cv2.imread(str(directory / 'map.pgm'), cv2.IMREAD_GRAYSCALE)
    second = cv2.imread(str(preview / 'current.pgm'), cv2.IMREAD_GRAYSCALE)
    if first is None or second is None or not np.array_equal(first, second):
        return False
    shutil.copyfile(preview / 'current.ply', directory / 'map.ply')
    return True


def recover_snapshot(database_path, save_directory, preview_directory, options):
    """只改数据库副本；校验、导出、摘要全部成功后才公开地图目录。"""
    # 此处延迟导入，避免纯配置/数据库检查依赖 ROS 初始化。
    from robot_navigation.mapping.snapshot_manager import apply_snapshot_permissions, snapshot_identity

    root = Path(save_directory).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    with open(root / '.mapping_recovery.lock', 'a') as lock:
        os.fchown(lock.fileno(), *snapshot_identity(root))
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('已有地图恢复正在进行，请等待完成') from exc
        candidates = [Path(database_path).expanduser().resolve()]
        candidates.append(Path(str(candidates[0]) + '.back'))
        assert_databases_closed(candidates)
        valid, errors = [], []
        for candidate in candidates:
            try:
                details = database_details(candidate)
                valid.append((details['last_node_stamp'], details['last_node_id'], details['node_count'], candidate, details))
            except (ValueError, OSError, sqlite3.DatabaseError) as exc:
                errors.append(str(exc))
        if not valid:
            raise ValueError('没有可恢复的有效数据库：' + '；'.join(errors))
        # 相同时优先活跃库，避免选择仅因复制而看起来更新的旧备份。
        _, _, _, source, details = max(valid, key=lambda item: item[:3])
        reserve = int(options['minimum_free_bytes'])
        if shutil.disk_usage(root).free < source.stat().st_size + reserve:
            raise ValueError('磁盘空间不足以复制数据库并导出地图，请先释放空间；原库未修改')
        now = datetime.now(ZoneInfo(str(options['snapshot_timezone'])))
        destination = root / now.strftime('map_%Y%m%d_%H%M%S_%f')
        with tempfile.TemporaryDirectory(prefix='.mapping-recovery-', dir=root) as temporary:
            staging = Path(temporary) / 'bundle'
            staging.mkdir()
            copy_database(source, staging / 'map.db')
            command = [str(options['export_executable']), '--map', '--opt', '2', '--save_in_db',
                       '--threads', '1', '--output', 'map', '--output_dir', str(staging)]
            command.append(str(staging / 'map.db'))
            commands = [command]
            if options['export_cloud']:
                # --save_in_db 会只把云写回数据库；三维需单独导出，且工具使用 _cloud 后缀。
                commands.append([str(options['export_executable']), '--cloud', '--opt', '2',
                                 '--threads', '1', '--output', 'map', '--output_dir', str(staging),
                                 '--decimation', str(options['cloud_decimation']),
                                 '--voxel', str(options['cloud_voxel_size']),
                                 '--density_radius', str(options['cloud_pose_radius']),
                                 '--density_angle', str(options['cloud_pose_angle']), str(staging / 'map.db')])
            try:
                deadline = time.monotonic() + float(options['export_timeout'])
                with open(staging / 'recovery.log', 'w') as log:
                    log.write('作用：记录离线地图恢复导出；使用方法：导出失败时检查本日志。\n')
                    log.flush()
                    for command in commands:
                        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True,
                                       timeout=max(0.0, deadline - time.monotonic()))
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
                with open(staging / 'recovery.log', 'rb') as log:
                    log.seek(max(0, (staging / 'recovery.log').stat().st_size - 3000))
                    tail = log.read().decode('utf-8', errors='replace')
                raise ValueError(f'离线地图导出失败，原库未修改：{exc}；{tail}') from exc
            if options['export_cloud']:
                generated_cloud = staging / 'map_cloud.ply'
                if not generated_cloud.is_file():
                    raise ValueError('已请求三维恢复但没有导出有效 PLY，请保留原库并检查深度数据')
                generated_cloud.rename(staging / 'map.ply')
            image = cv2.imread(str(staging / 'map.pgm'), cv2.IMREAD_GRAYSCALE)
            if image is None or not (staging / 'map.yaml').is_file():
                raise ValueError('数据库尚未保存可导出的地图，请保留原库检查；未生成伪地图')
            settings = yaml.safe_load((staging / 'map.yaml').read_text())
            if settings.get('image') != 'map.pgm' or float(settings.get('resolution', 0)) <= 0:
                raise ValueError('导出地图 YAML 无效')
            cloud_source = 'database' if (staging / 'map.ply').is_file() else 'unavailable'
            if cloud_source == 'unavailable':
                try:
                    if attach_matching_cloud(staging, preview_directory, database_path):
                        cloud_source = 'matching_cached_map'
                except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError):
                    # 缓存损坏不能阻止救出已校验的二维地图和数据库。
                    pass
            metadata = {
                'created_at': now.isoformat(), 'map_frame': 'map', 'database': 'map.db',
                'map_width': int(image.shape[1]), 'map_height': int(image.shape[0]),
                'resolution': float(settings['resolution']), 'recovered': True,
                'source_database': str(source), 'source_validation_errors': errors,
                'cloud_available': cloud_source != 'unavailable', 'cloud_source': cloud_source,
                **details,
                'files_sha256': {p.name: file_digest(p) for p in staging.iterdir()
                                 if p.suffix in ('.db', '.yaml', '.pgm', '.ply')},
            }
            (staging / 'map.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
            validate_bundle(staging / 'map.yaml', staging / 'map.db')
            apply_snapshot_permissions(staging, *snapshot_identity(root))
            staging.rename(destination)
        return str(destination / 'map.yaml')


def main():
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description='SLAM 退出后恢复保存地图，不启动相机、底盘或在线建图')
    parser.add_argument('--config', default=str(Path(get_package_share_directory('robot_navigation')) /
                                              'config' / 'mapping_recovery.yaml'))
    parser.add_argument('--database-path', help='残留活跃数据库路径，也会检查同名 .back')
    parser.add_argument('--save-directory', help='恢复地图根目录')
    parser.add_argument('--preview-directory', help='退出前缓存地图目录')
    arguments = parser.parse_args()
    try:
        options = load_options(arguments.config)
        result = recover_snapshot(arguments.database_path or options['database_path'],
                                  arguments.save_directory or options['save_directory'],
                                  arguments.preview_directory or options['preview_directory'], options)
    except (ValueError, OSError, RuntimeError, sqlite3.DatabaseError, yaml.YAMLError) as exc:
        parser.exit(1, f'恢复地图失败：{exc}\n')
    print(f'地图恢复保存成功：{result}', flush=True)


if __name__ == '__main__':
    main()
