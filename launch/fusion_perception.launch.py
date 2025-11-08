from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('fusion_cpp')
    
    # 配置文件路径
    config_file = os.path.join(pkg_dir, 'config', 'fusion_config.yaml')
    
    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 融合感知节点
    fusion_node = Node(
        package='fusion_cpp',
        executable='fusion_perception_node',
        name='fusion_perception_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # 可以在这里添加topic重映射
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        fusion_node
    ])
