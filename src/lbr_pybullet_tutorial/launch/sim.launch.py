"""Start only the reusable PyBullet simulator, independently of controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false'],
                              description='Open the interactive PyBullet visualizer'),
        DeclareLaunchArgument('sim_hz', default_value='240.0'),
        DeclareLaunchArgument('render_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('show_ee_trace', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('command_timeout', default_value='0.5'),
        DeclareLaunchArgument('position_command_topic', default_value='/iiwa7/joint_desired'),
        DeclareLaunchArgument('joint_state_topic', default_value='/iiwa7/joint_states'),
        DeclareLaunchArgument('ee_pose_topic', default_value='/lbr/ee_pose'),
        Node(package='lbr_pybullet_tutorial', executable='sim_node', output='screen',
             parameters=[{
                 'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool),
                 'render_camera': ParameterValue(LaunchConfiguration('render_camera'), value_type=bool),
                 'show_ee_trace': ParameterValue(LaunchConfiguration('show_ee_trace'), value_type=bool),
                 'sim_hz': ParameterValue(LaunchConfiguration('sim_hz'), value_type=float),
                 'command_timeout': ParameterValue(LaunchConfiguration('command_timeout'),
                                                   value_type=float),
                 'position_command_topic': LaunchConfiguration('position_command_topic'),
                 'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                 'ee_pose_topic': LaunchConfiguration('ee_pose_topic'),
             }]),
    ])
