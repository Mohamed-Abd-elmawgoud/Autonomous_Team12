import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='teleop',
        description='Control mode: teleop or auto'
    )

    mode = LaunchConfiguration('mode')


    velocity_arg = DeclareLaunchArgument(
    'velocity', default_value='1.5',
    description='Velocity for auto mode (m/s)'
    )

    steering_arg = DeclareLaunchArgument(
    'steering', default_value='0.8',
    description='Steering angle for auto mode (-1 to +1)'
    )

    # ── Package paths ─────────────────────────────────────────────────────
    gazebo_pkg   = get_package_share_directory('gazebo_ackermann_steering_vehicle')
    team_pkg     = get_package_share_directory('autonomous_systems_project_team_12')

    vehicle_params_path = os.path.join(gazebo_pkg, 'config', 'parameters.yaml')

    # ── Nodes ─────────────────────────────────────────────────────────────

    # Always running
    vehicle_controller_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='vehicle_controller',
        parameters=[vehicle_params_path],
        output='screen'
    )

    esp_bridge_node = Node(
        package='autonomous_systems_project_team_12',
        executable='esp_bridge',
        output='screen'
    )

    # # Teleop mode only
    # teleop_vehicle_controller_node = Node(
    #     package='autonomous_systems_project_team_12',
    #     executable='teleop_vehicle_controller',
    #     output='screen',
    #     condition=IfCondition(
    #         PythonExpression(["'", mode, "' == 'teleop'"])
    #     )
    # )

    # Auto mode only
    olr_node = Node(
        package='autonomous_systems_project_team_12',
        executable='olr_node',
        output='screen',
        parameters=[{
            'velocity':       LaunchConfiguration('velocity'),
            'steering_angle': LaunchConfiguration('steering'),
        }],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'auto'"])
        )
    )

    return LaunchDescription([
        mode_arg,
        velocity_arg,
        steering_arg,
        vehicle_controller_node,
        esp_bridge_node,
        # teleop_vehicle_controller_node,
        olr_node,
    ])