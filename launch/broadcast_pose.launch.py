import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'device_id',
            default_value=launch.substitutions.EnvironmentVariable(
                'DRONE_DEVICE_ID',
                default_value='undefined'
            ),
            description='Drone unique identifier.'
        ),
        launch.actions.DeclareLaunchArgument(
            'right_of_way',
            default_value=launch.substitutions.EnvironmentVariable(
                'RIGHT_OF_WAY',
                default_value='0'
            ),
            description='Right of way identifier.'
        ),
        launch_ros.actions.Node(
            package='broadcast_pose',
            namespace=os.getenv('DRONE_DEVICE_ID'),
            executable='broadcast_pose_node',
            name='broadcast_pose',
            parameters=[
                {
                    "device_id": launch.substitutions.LaunchConfiguration('device_id'),
                    "right_of_way": launch.substitutions.LaunchConfiguration('right_of_way'),
                }
            ]
        )
    ])
