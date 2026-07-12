from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    package_name = 'rover_arm_description'
    pkg_share = get_package_share_directory(package_name)

    # Inject paths into Gazebo environment variables for mesh resolution
    gazebo_model_path = os.path.dirname(pkg_share)
    for env_var in ['GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH']:
        if env_var in os.environ:
            os.environ[env_var] += os.pathsep + gazebo_model_path
        else:
            os.environ[env_var] = gazebo_model_path

    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'urdf', 'rover_arm_sim.xacro'
    ])
    
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str
        )
    }

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description,
                     {"use_sim_time": True},]    #To use simulation time for the robot state publisher ie../clock topic from Gazebo
    )

    # 2. Gazebo World Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Gazebo Spawner
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
          # '-string', Command([FindExecutable(name='xacro'), ' ', xacro_file]),
            '-topic', 'robot_description',    #Replaced string with topic
            '-name', 'rover_arm',
            '-z', '0.1'
        ]
    )

    # 4. Bridge Node for Clock, Transforms, and Gripper Camera
    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # ADDED: Translates the camera stream from Gazebo -> ROS 2
            '/gripper_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    # 5. RViz2
#node_rviz = Node(
#     package='rviz2',
#    executable='rviz2',
# )
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{"use_sim_time": True}],
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'rviz',
                'rover.rviz'
            ])
        ]
    )

    # ==================== TELEOP & CONTROLLER ADDITIONS ====================

    # 6. Physical Joystick Driver Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    # 7. Your Custom C++ Teleop Control Node
    sim_control_node = Node(
        package=package_name,
        executable='sim_control_node',
        name='sim_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. ros2_control Spawners
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    # Delay loading the controllers until after your robot has actually spawned in Gazebo
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_spawn_entity,
            on_start=[joint_state_broadcaster, arm_controller],
        )
    )
    # =======================================================================

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo_launch,
        node_spawn_entity,
        node_gz_bridge,
        node_rviz,
        joy_node,
        sim_control_node,
        delay_controllers
    ])