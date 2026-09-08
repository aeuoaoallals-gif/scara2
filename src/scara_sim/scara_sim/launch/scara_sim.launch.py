from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_scara_sim = get_package_share_directory('scara_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_scara_sim, 'worlds', 'empty.world')
    xacro_file = os.path.join(pkg_scara_sim, 'description', 'scara.urdf.xacro')
    controllers_yaml = os.path.join(pkg_scara_sim, 'config', 'scara_controllers.yaml')
    rviz_config = os.path.join(pkg_scara_sim, 'rviz', 'scara.rviz')

    # Получаем URDF из xacro
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    urdf_str = doc.toprettyxml(indent='  ')

    robot_description = {'robot_description': urdf_str}

    # Запуск Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 1 {world_file}'}.items()
    )

    # Публикация robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': True,
                'publish_frequency': 60.0, 
            }
        ],
        
    )

    # Спавн робота
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'scara', '-allow_renaming', 'true'],
        output='screen',
        parameters=[{'use_sim_time': True}]
        
    )

    # Spawner'ы контроллеров
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': True}]
        
    )

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '--param-file', controllers_yaml],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Публикатор моментов
    effort_publisher = Node(
        package='scara_sim',
        executable='effort_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Публикатор моментов
    trajectory_generator = Node(
        package='scara_sim',
        executable='trajectory_generator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Мост для /clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
        output='screen',
        parameters=[{'use_sim_time': True}]
        
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
        
    )
    
    trajectory_visualizer = Node(
        package='scara_sim',
        executable='trajectory_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_frequency': 60.0, 
        }]
        
    )
    real_trajectory_visualizer = Node(
        package='scara_sim',
        executable='real_trajectory_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}]  
    )
    data_logger_node = Node(
        package='scara_sim',             # Имя твоего пакета
        executable='data_logger',        # Имя, которое мы указали в setup.py
        name='scara_data_logger',
        output='screen',
        parameters=[{
            'csv_filename': 'scara_data.csv',
            'base_frame': 'base_link',          # ЗАМЕНИ на имя базового линка из твоего URDF
            'ee_frame': 'tool0',                # ЗАМЕНИ на имя конечного линка (инструмента) из URDF
            'joint_states_topic': '/joint_states',
            'ideal_state_topic': '/joint_trajectory_controller/state' # ЗАМЕНИ, если топик контроллера называется иначе
        }]
    )

    move_camera = ExecuteProcess(
        cmd=[
            'ign', 'service', '-s', '/gui/move_to/pose',
            '--reqtype', 'ignition.msgs.GUICamera',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req',
            'pose: {position: {x: 0.0, y: 0.0, z: 0.7}, orientation: {x: 0.5, y: 0.5, z: -0.5, w: 0.5}}'
        ],
        output='screen'
    )



    return LaunchDescription([
        trajectory_generator,
        gz_sim,
        robot_state_publisher,
        bridge,
        spawn_robot,
        rviz,    
        trajectory_visualizer,
        real_trajectory_visualizer,  
        data_logger_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[jsb_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[effort_controller_spawner],
            )
        ),
        TimerAction(period=5.0, actions=[move_camera]),
    ])