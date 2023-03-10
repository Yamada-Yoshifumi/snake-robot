import os
import pathlib
import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
#from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('snake_robot')
    world_file = os.path.join(package_dir, 'worlds', 'snake_basic_crates_and_barrels_1.wbt')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'salamander.urdf')).read_text()
    webots = WebotsLauncher(
        world=world_file,
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        #additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'salamander'},
        additional_env={'WEBOTS_CONTROLLER_URL': 'ipc://1234/salamander'},
        parameters=[
            {'robot_description': robot_description},
        ],
    )

    trajectory_recorder = Node(
        package='snake_robot',
        executable='trajectory_recording',
        output='screen',
        parameters=[
            {'world_file': world_file},
        ],
    )

    return LaunchDescription([
        webots,
        trajectory_recorder,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        
    ])
