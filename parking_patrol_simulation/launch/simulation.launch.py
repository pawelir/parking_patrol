from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    parking_world_path = PathJoinSubstitution(
        [FindPackageShare("parking_patrol_simulation"), "world", "parking.world"]
    )

    world_argument = DeclareLaunchArgument(
        "world",
        default_value=parking_world_path,
        description="Simulation world",
    )

    SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger")

    turtlebot3_simulation_launch = create_include_launch(
        package="turtlebot3_gazebo",
        rel_launch_path="launch/empty_world.launch.py",
        arguments={"world": LaunchConfiguration("world")},
    )

    actions = [world_argument, turtlebot3_simulation_launch]

    return LaunchDescription(actions)


def create_include_launch(package: str, rel_launch_path: str, arguments: dict):
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(package), rel_launch_path])]
        ),
        launch_arguments=arguments.items(),
    )
    return included_launch
