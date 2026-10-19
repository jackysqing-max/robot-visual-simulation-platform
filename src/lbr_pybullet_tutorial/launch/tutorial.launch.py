"""LBR JTC control with a local or separately launched PyBullet ROS node."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from lbr_pybullet_tutorial.model import INITIAL_Q, robot_description


def launch_setup(context):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    backend = LaunchConfiguration('sim_backend').perform(context)
    share = Path(get_package_share_directory('lbr_pybullet_tutorial'))
    description = robot_description()
    simulation = []
    if backend == 'tutorial':
        simulation = [Node(package='lbr_pybullet_tutorial', executable='sim_node',
                           output='screen', parameters=[
                               {'gui': gui, 'initial_q': list(INITIAL_Q)}])]
    elif backend == 'existing':
        simulation = [Node(package='pybullet_ros2_sim', executable='iiwa_pybullet_sim_node',
                           output='screen', parameters=[{
                       'gui': gui, 'initial_control_mode': 1, 'use_goto': False,
                       'reset_to_init_on_start': True, 'init_q': list(INITIAL_Q),
                       'enable_rgbd_camera': False,
                   }])]
    elif backend != 'external':
        raise ValueError('sim_backend must be tutorial, external, or existing')
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   namespace='lbr', output='screen', parameters=[
                       {'robot_description': ParameterValue(description, value_type=str)},
                       str(share/'config/lbr_controllers.upstream.yaml'),
                       str(share/'config/tutorial_controllers.yaml'),
                   ])
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '-c', '/lbr/controller_manager',
                                  '--controller-manager-timeout', '30'], output='screen')
    controller = Node(package='controller_manager', executable='spawner',
                      arguments=['joint_trajectory_controller', '-c', '/lbr/controller_manager',
                                 '--controller-manager-timeout', '30'], output='screen')
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                           namespace='lbr', output='screen',
                           parameters=[{'robot_description': description}])
    motion = Node(package='lbr_pybullet_tutorial', executable='motion_node', output='screen')
    return simulation + [manager, state_publisher, broadcaster,
            RegisterEventHandler(OnProcessExit(target_action=broadcaster,
                                               on_exit=[controller])), motion]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('sim_backend', default_value='tutorial',
                              choices=['tutorial', 'external', 'existing'],
                              description='tutorial: bundled simulator; external: existing ROS node; '
                                          'existing: optional legacy simulator package'),
        OpaqueFunction(function=launch_setup),
    ])
