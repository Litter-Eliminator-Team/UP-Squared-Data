from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription([
        # ML MODEL
        Node(
            package='perception_navigation',
            namespace='litterelim',
            executable='image_subber',
            name='ml_model'
        )
    ])

    INTEL_CAMERA_LAUNCH = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        # launch_arguments={
        #     'turtlesim_ns': 'turtlesim2',
        #     'use_provided_red': 'True',
        #     'new_background_r': TextSubstitution(text=str(colors['background_r']))
        # }.items()
    )

    ld.add_action(INTEL_CAMERA_LAUNCH)
    return ld
