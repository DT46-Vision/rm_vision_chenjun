import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

# 加载launch参数
launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

# 使用xacro生成机器人描述（URDF）
robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
    ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

# 定义robot_state_publisher节点
robot_state_publisher = Node(
    package='robot_state_publisher',  # 包名
    executable='robot_state_publisher',  # 可执行文件名
    parameters=[{'robot_description': robot_description,  # 机器人描述（URDF）
                 'publish_frequency': 1000.0}]  # 发布频率
)

# 节点参数文件路径
node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

# 定义armor_tracker节点
tracker_node = Node(
    package='armor_tracker',  # 包名
    executable='armor_tracker_node',  # 可执行文件名
    output='both',  # 输出到终端和日志
    emulate_tty=True,  # 模拟TTY以获得更好的日志格式
    parameters=[node_params],  # 使用节点参数文件
    ros_arguments=['--log-level', 'armor_tracker:=' + launch_params['tracker_log_level']],  # 设置日志级别
)
