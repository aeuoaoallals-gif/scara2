from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Публикатор моментов
    effort_publisher = Node(
        package='scara_sim',
        executable='effort_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        effort_publisher,
    ])