import os
import sys
from ament_index_python.packages import get_package_share_directory

# 将'rm_vision_bringup'包中的launch目录添加到系统路径
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():
    # 从common模块导入必要的参数和节点定义
    from common import launch_params, robot_state_publisher, node_params, tracker_node
    from launch_ros.actions import Node
    from launch import LaunchDescription

    # 定义装甲检测器节点
    detector_node = Node(
        package='armor_detector',  # 包名
        executable='armor_detector_node',  # 可执行文件名
        emulate_tty=True,  # 模拟TTY以获得更好的日志格式
        output='both',  # 输出到终端和日志
        parameters=[node_params],  # 使用通用节点参数
        arguments=['--ros-args', '--log-level', 'armor_detector:=' + launch_params['detector_log_level']],  # 设置日志级别
    )

    # 返回Launch描述对象，包含机器人状态发布器、装甲检测器节点和跟踪器节点
    return LaunchDescription([
        robot_state_publisher,
        detector_node,
        tracker_node,
    ])
