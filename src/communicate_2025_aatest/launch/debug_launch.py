'''
需要先创建保存日志的文件路径，不然无法保存,但是不影响调试，只是不会保存日志
如果没有 gnome-terminal ,可以使用 xterm 替代，直接将 prefix 里面的 gnome-terminal 替换成 xterm
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('communicate_2025_aatest'),
        'config', 'config.yaml'
    )

    load_nodes=GroupAction(
        actions=[
            Node(
                package='communicate_2025_aatest',
                executable='communicate_2025_aatest_node',
                output='screen',
                parameters=[config],
                emulate_tty=True,
                prefix='gnome-terminal -- gdb -ex "set logging file ./debug/gdb/communicate_2025_aatest_gdb.txt" -ex "set logging enabled" --args'
            ),
        ]
    )
    ld = LaunchDescription()
    ld.add_action(load_nodes)
    return ld