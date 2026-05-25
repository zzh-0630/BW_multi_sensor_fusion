from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'livox_custom_downsample_demo'

    leaf_size_arg = DeclareLaunchArgument(
        'leaf_size',
        default_value='0.3',
        description='VoxelGrid leaf size in meters.'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar',
        description='Input Livox CustomMsg topic.'
    )

    raw_output_topic_arg = DeclareLaunchArgument(
        'raw_output_topic',
        default_value='/livox/lidar_raw_pointcloud2',
        description='Output raw converted PointCloud2 topic.'
    )

    downsampled_output_topic_arg = DeclareLaunchArgument(
        'downsampled_output_topic',
        default_value='/livox/lidar_downsampled',
        description='Output downsampled PointCloud2 topic.'
    )

    default_frame_id_arg = DeclareLaunchArgument(
        'default_frame_id',
        default_value='livox_frame',
        description='Default frame id when Livox message header frame_id is empty.'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'livox_downsample.rviz'
        ]),
        description='RViz2 config file path.'
    )

    downsample_node = Node(
        package=package_name,
        executable='livox_custom_voxel_downsample',
        name='livox_custom_voxel_downsample_node',
        output='screen',
        parameters=[{
            'leaf_size': LaunchConfiguration('leaf_size'),
            'input_topic': LaunchConfiguration('input_topic'),
            'raw_output_topic': LaunchConfiguration('raw_output_topic'),
            'downsampled_output_topic': LaunchConfiguration('downsampled_output_topic'),
            'default_frame_id': LaunchConfiguration('default_frame_id'),
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        leaf_size_arg,
        input_topic_arg,
        raw_output_topic_arg,
        downsampled_output_topic_arg,
        default_frame_id_arg,
        rviz_config_arg,
        downsample_node,
        rviz_node,
    ])