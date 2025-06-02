from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/robot_cam/colored_map@sensor_msgs/msg/Image@ignition.msgs.Image'
            ],
            output='screen'
        )
    ])
