from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare("adaptive_controller")

    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "exoskeleton.urdf.xacro"])

    # FIX: wrap in ParameterValue(value_type=str) — required in ROS2 Jazzy
    # Without this, launch tries to parse the xacro XML output as YAML and crashes.
    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "exoskeleton.rviz"])

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz2"
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="true",
        description="Use joint_state_publisher_gui (slider panel)"
    )

    return LaunchDescription([
        use_rviz_arg,
        use_gui_arg,

        # 1. robot_state_publisher
        #    Reads robot_description, subscribes to /joint_states,
        #    broadcasts TF for every link frame.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description_content,
                "use_sim_time": False,
            }],
        ),

        # 2. joint_state_publisher_gui
        #    Publishes /joint_states for all 8 revolute joints.
        #    robot_state_publisher needs these to build the full TF tree.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_gui")),
            parameters=[{"use_sim_time": False}],
        ),

        # 3. RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", rviz_config],
        ),
    ])