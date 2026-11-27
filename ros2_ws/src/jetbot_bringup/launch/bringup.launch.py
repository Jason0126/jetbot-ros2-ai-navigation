
import os

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackagePrefix

from ament_index_python.packages import (
    get_package_share_directory,
)


def generate_launch_description():

    # ========================================================
    # 1. Package paths
    # ========================================================

    bringup_dir = get_package_share_directory(
        'jetbot_bringup'
    )

    slam_dir = get_package_share_directory(
        'slam_toolbox'
    )

    nav2_dir = get_package_share_directory(
        'nav2_bringup'
    )

    slam_params = os.path.join(
        bringup_dir,
        'config',
        'slam',
        'mapper_params_online_async.yaml'
    )

    nav2_params = os.path.join(
        bringup_dir,
        'config',
        'nav2',
        'nav2_params.yaml'
    )

    # ========================================================
    # 2. Launch arguments
    # ========================================================

    launch_arguments = [

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'enable_slam',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'enable_nav2',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'enable_monitor',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'enable_hmi',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'enable_yolo',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'nav2_delay',
            default_value='5.0'
        ),

        DeclareLaunchArgument(
            'yolo_python',
            default_value=os.path.expanduser(
                '~/jetbot_yolo_venv/bin/python'
            )
        ),

        DeclareLaunchArgument(
            'yolo_weights',
            default_value='yolo11n.pt'
        ),

        DeclareLaunchArgument(
            'yolo_device',
            default_value='cpu'
        ),

        DeclareLaunchArgument(
            'yolo_imgsz',
            default_value='320'
        ),

        DeclareLaunchArgument(
            'yolo_conf',
            default_value='0.35'
        ),

        DeclareLaunchArgument(
            'yolo_max_fps',
            default_value='3'
        ),
    ]

    # ========================================================
    # 3. SLAM Toolbox
    # ========================================================

    slam_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                slam_dir,
                'launch',
                'online_async_launch.py'
            )
        ),

        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': LaunchConfiguration(
                'use_sim_time'
            ),
        }.items(),

        condition=IfCondition(
            LaunchConfiguration('enable_slam')
        ),
    )

    # ========================================================
    # 4. Nav2
    # ========================================================

    nav2_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_dir,
                'launch',
                'navigation_launch.py'
            )
        ),

        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': LaunchConfiguration(
                'use_sim_time'
            ),
            'autostart': 'true',
        }.items(),
    )

    delayed_nav2 = TimerAction(

        period=LaunchConfiguration('nav2_delay'),

        actions=[nav2_launch],

        condition=IfCondition(
            LaunchConfiguration('enable_nav2')
        ),
    )

    # ========================================================
    # 5. Robot State Monitor
    # ========================================================

    monitor_node = Node(

        package='jetbot_monitor',

        executable='state_monitor',

        output='screen',

        condition=IfCondition(
            LaunchConfiguration('enable_monitor')
        ),
    )

    # ========================================================
    # 6. Flask HMI
    # ========================================================

    hmi_node = Node(

        package='jetbot_hmi',

        executable='flask_hmi',

        output='screen',

        condition=IfCondition(
            LaunchConfiguration('enable_hmi')
        ),
    )

    # ========================================================
    # 7. YOLO Perception
    # ========================================================

    yolo_executable = PathJoinSubstitution([
        FindPackagePrefix('jetbot_perception'),
        'lib',
        'jetbot_perception',
        'yolo_node',
    ])

    yolo_node = ExecuteProcess(

        cmd=[
            LaunchConfiguration('yolo_python'),

            yolo_executable,

            '--weights',
            LaunchConfiguration('yolo_weights'),

            '--device',
            LaunchConfiguration('yolo_device'),

            '--imgsz',
            LaunchConfiguration('yolo_imgsz'),

            '--conf',
            LaunchConfiguration('yolo_conf'),

            '--max-fps',
            LaunchConfiguration('yolo_max_fps'),
        ],

        name='yolo_perception',

        output='screen',

        condition=IfCondition(
            LaunchConfiguration('enable_yolo')
        ),
    )

    # ========================================================
    # 8. Launch all components
    # ========================================================

    return LaunchDescription(

        launch_arguments + [

            slam_launch,

            monitor_node,

            hmi_node,

            yolo_node,

            delayed_nav2,
        ]
    )
