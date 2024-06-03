import os
import sys
from ament_index_python.packages import get_package_share_directory

# 将'rm_vision_bringup'包中的launch目录添加到系统路径
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():
    # 从common模块导入必要的参数和节点定义
    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    # 定义函数以生成相机节点
    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],  # 使用通用节点参数
            extra_arguments=[{'use_intra_process_comms': True}]  # 使用进程内通信
        )

    # 定义函数以生成包含相机和装甲检测器的容器
    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,  # 添加相机节点
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],  # 使用通用节点参数
                    extra_arguments=[{'use_intra_process_comms': True}]  # 使用进程内通信
                )
            ],
            output='both',  # 输出到终端和日志
            emulate_tty=True,  # 模拟TTY以获得更好的日志格式
            ros_arguments=['--ros-args', '--log-level', 'armor_detector:=' + launch_params['detector_log_level']],  # 设置日志级别
            on_exit=Shutdown(),  # 当容器退出时关闭所有组件
        )

    # 创建两种相机节点
    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    # 根据配置选择相应的相机检测器容器
    if launch_params['camera'] == 'hik':
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif launch_params['camera'] == 'mv':
        cam_detector = get_camera_detector_container(mv_camera_node)

    # 创建串行驱动节点
    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',  # 输出到终端和日志
        emulate_tty=True,  # 模拟TTY以获得更好的日志格式
        parameters=[node_params],  # 使用通用节点参数
        on_exit=Shutdown(),  # 当节点退出时关闭
        ros_arguments=['--ros-args', '--log-level', 'serial_driver:=' + launch_params['serial_log_level']],  # 设置日志级别
    )

    # 定义延时启动的计时器动作（1.5秒后启动串行驱动节点）
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    # 定义延时启动的计时器动作（2.0秒后启动跟踪器节点）
    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    # 返回Launch描述对象，包含机器人状态发布器、相机检测器容器、延时启动的串行驱动节点和跟踪器节点
    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        delay_serial_node,
        delay_tracker_node,
    ])