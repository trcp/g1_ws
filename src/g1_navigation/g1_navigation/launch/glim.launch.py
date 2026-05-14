#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os


DUMP_TO_PCD_SCRIPT = r'''
import os
import struct
import sys


DUMP_DIR = '/tmp/dump'
MAP_DIR = os.path.expanduser('~/colcon_ws/map')
IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def map_stem():
    stem = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else 'map'
    stem = os.path.basename(stem)
    if stem.endswith('.pcd'):
        stem = stem[:-4]
    return stem or 'map'


def load_t_world_origin(data_path):
    if not os.path.isfile(data_path):
        return IDENTITY

    with open(data_path, 'r', encoding='utf-8') as data:
        lines = data.readlines()

    for index, line in enumerate(lines):
        if line.strip() == 'T_world_origin:':
            return [
                [float(value) for value in lines[index + row + 1].split()]
                for row in range(4)
            ]

    return IDENTITY


def transform_point(matrix, point):
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
    )


def iter_submap_points(submap_dir):
    points_path = os.path.join(submap_dir, 'points_compact.bin')
    intensities_path = os.path.join(submap_dir, 'intensities_compact.bin')
    if not os.path.isfile(points_path):
        return

    matrix = load_t_world_origin(os.path.join(submap_dir, 'data.txt'))

    with open(points_path, 'rb') as points_file:
        points_blob = points_file.read()

    intensities_blob = b''
    if os.path.isfile(intensities_path):
        with open(intensities_path, 'rb') as intensities_file:
            intensities_blob = intensities_file.read()

    num_points = len(points_blob) // 12
    num_intensities = len(intensities_blob) // 4

    for index in range(num_points):
        point = struct.unpack_from('<fff', points_blob, index * 12)
        intensity = 0.0
        if index < num_intensities:
            intensity = struct.unpack_from('<f', intensities_blob, index * 4)[0]
        yield (*transform_point(matrix, point), intensity)


def collect_points():
    if not os.path.isdir(DUMP_DIR):
        return []

    submap_dirs = [
        os.path.join(DUMP_DIR, name)
        for name in sorted(os.listdir(DUMP_DIR))
        if name.isdigit() and os.path.isdir(os.path.join(DUMP_DIR, name))
    ]

    points = []
    for submap_dir in submap_dirs:
        points.extend(iter_submap_points(submap_dir) or [])
    return points


def write_pcd(points, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    header = [
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z intensity',
        'SIZE 4 4 4 4',
        'TYPE F F F F',
        'COUNT 1 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {len(points)}',
        'DATA binary',
    ]

    tmp_path = path + '.tmp'
    with open(tmp_path, 'wb') as pcd:
        pcd.write(('\n'.join(header) + '\n').encode('ascii'))
        for point in points:
            pcd.write(struct.pack('<ffff', *point))
    os.replace(tmp_path, path)


def main():
    points = collect_points()
    if not points:
        print(f'No GLIM dump points found under {DUMP_DIR}')
        return

    stem = map_stem()
    map_path = os.path.join(MAP_DIR, f'{stem}.pcd')
    write_pcd(points, map_path)
    print(f'Saved GLIM dump map: {map_path} ({len(points)} points)')


if __name__ == '__main__':
    main()
'''


GLIM_WRAPPER_SCRIPT = r'''
set -u

CONFIG_PATH="$1"
USE_SIM_TIME="$2"
MAP_NAME="$3"
MIN_HEIGHT="$4"
MAX_HEIGHT="$5"
MAP_DIR="$HOME/colcon_ws/map"
POINTCLOUD_TO_2DMAP="$HOME/colcon_ws/build/pointcloud_to_2dmap/pointcloud_to_2dmap"

export DISPLAY=:1001

map_stem() {
  local stem
  stem="$(basename "$MAP_NAME")"
  stem="${stem%.pcd}"
  if [ -z "$stem" ]; then
    stem="map"
  fi
  printf '%s\n' "$stem"
}

save_dump_to_pcd() {
  python3 -c "$DUMP_TO_PCD_SCRIPT" "$MAP_NAME" || true
}

save_pcd_to_2d_map() {
  local stem
  local pcd_file

  stem="$(map_stem)"
  pcd_file="${stem}.pcd"

  if [ ! -f "${MAP_DIR}/${pcd_file}" ]; then
    echo "Skip 2D map export: ${MAP_DIR}/${pcd_file} does not exist"
    return 0
  fi

  if [ ! -x "$POINTCLOUD_TO_2DMAP" ]; then
    echo "Skip 2D map export: $POINTCLOUD_TO_2DMAP is not executable"
    return 0
  fi

  (
    cd "$MAP_DIR" && \
    "$POINTCLOUD_TO_2DMAP" "$pcd_file" . --min_height "$MIN_HEIGHT" --max_height "$MAX_HEIGHT"
  ) || true
}

stop_glim() {
  if [ -n "${GLIM_PID:-}" ] && kill -0 "$GLIM_PID" 2>/dev/null; then
    kill -INT "$GLIM_PID" 2>/dev/null || true
    wait "$GLIM_PID" 2>/dev/null || true
  fi
}

cleanup() {
  status=$?
  stop_glim
  save_dump_to_pcd
  save_pcd_to_2d_map
  exit "$status"
}

handle_signal() {
  trap - INT TERM
  stop_glim
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

ros2 run glim_ros glim_rosnode --ros-args \
  -p "config_path:=${CONFIG_PATH}" \
  -p "use_sim_time:=${USE_SIM_TIME}" &
GLIM_PID=$!
wait "$GLIM_PID"
'''


def generate_launch_description():
    ld = LaunchDescription()


    default_config_path = os.path.join(get_package_share_directory('g1_navigation'), 'config')


    # GLIM 用
    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # 生成されるマップの設定
    map_name = LaunchConfiguration('map_name')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')


    declare_config_path = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='path to GLIM config directory'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='use sim time'
    )
    declare_map = DeclareLaunchArgument(
        'map_name', default_value='map',
        description='PCD map file name without extension'
    )
    declare_min_height = DeclareLaunchArgument(
        'min_height', default_value='0.0',
        description='Minimum height of map from 3d PCD map to 2D png map.'
    )
    declare_max_height = DeclareLaunchArgument(
        'max_height', default_value='4.0',
        description='Max height of map from 3d PCD map to 2D png map.'
    )
    ld.add_action(declare_config_path)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_min_height)
    ld.add_action(declare_max_height)


    glim_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            GLIM_WRAPPER_SCRIPT,
            'glim_wrapper',
            config_path,
            use_sim_time,
            map_name,
            min_height,
            max_height,
        ],
        name='glim_rosnode',
        output='screen',
        additional_env={
            'DISPLAY': ':1001',
            'DUMP_TO_PCD_SCRIPT': DUMP_TO_PCD_SCRIPT,
        },
        sigterm_timeout='30',
        sigkill_timeout='30',
    )
    dump_on_shutdown = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                'set -u; '
                'MAP_NAME="$1"; MIN_HEIGHT="$2"; MAX_HEIGHT="$3"; '
                'MAP_DIR="$HOME/colcon_ws/map"; '
                'POINTCLOUD_TO_2DMAP="$HOME/colcon_ws/build/pointcloud_to_2dmap/pointcloud_to_2dmap"; '
                'stem="$(basename "$MAP_NAME")"; stem="${stem%.pcd}"; '
                'if [ -z "$stem" ]; then stem="map"; fi; '
                'sleep 5; '
                'python3 -c "$DUMP_TO_PCD_SCRIPT" "$MAP_NAME" || true; '
                'pcd_file="${stem}.pcd"; '
                'if [ ! -f "${MAP_DIR}/${pcd_file}" ]; then '
                '  echo "Skip 2D map export: ${MAP_DIR}/${pcd_file} does not exist"; exit 0; '
                'fi; '
                'if [ ! -x "$POINTCLOUD_TO_2DMAP" ]; then '
                '  echo "Skip 2D map export: $POINTCLOUD_TO_2DMAP is not executable"; exit 0; '
                'fi; '
                'cd "$MAP_DIR" && '
                '"$POINTCLOUD_TO_2DMAP" "$pcd_file" . --min_height "$MIN_HEIGHT" --max_height "$MAX_HEIGHT" || true'
            ),
            'dump_on_shutdown',
            map_name,
            min_height,
            max_height,
        ],
        name='glim_dump_to_pcd',
        output='screen',
        additional_env={
            'DISPLAY': ':1001',
            'DUMP_TO_PCD_SCRIPT': DUMP_TO_PCD_SCRIPT,
        },
    )

    ld.add_action(glim_process)
    ld.add_action(RegisterEventHandler(
        OnShutdown(on_shutdown=[dump_on_shutdown])
    ))


    return ld
