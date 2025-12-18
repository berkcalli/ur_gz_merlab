from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)

def launch_setup(context, *args, **kwargs):
    gz_spawn_object = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            "/home/bcalli/datasets/ycb-models/002_master_chef_can/master_chef_can.sdf",
            "-name",
            "target_object",
            "-x", "0.6",
            "-y", "0.0",
            "-z", "1.0",
            "-allow_renaming",
            "true", 
        ],)
    gz_spawn_table = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            "/home/bcalli/datasets/gazebo_objects/FoodCourtTable1/model.sdf",
            "-name",
            "table",
            "-x", "0.6",
            "-y", "0.0",
            "-z", "0.0",
            "-allow_renaming",
            "true", 
        ],)
    
    gz_spawn_camera = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            [FindPackageShare("ur_simulation_gz"), "/urdf/camera_rgbd.urdf"],
            "-name",
            "camera_sensor",
            "-x", "0.5",
            "-y", "0.0",
            "-z", "1.3",
            "-allow_renaming",
            "true", 
        ],)
    gz_sim_bridge_camera_rgb_image = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    gz_sim_bridge_camera_depth_image = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    nodes_to_start = [
        gz_spawn_object,
        gz_spawn_table,
        gz_spawn_camera,
        gz_sim_bridge_camera_rgb_image,
        gz_sim_bridge_camera_depth_image,
    ]
    return nodes_to_start

def generate_launch_description():
    launch_dir = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "launch"]
    )
    return LaunchDescription([

        # include another launch file
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'ur_sim_control.launch.py'])
        ),
        ]+
        [OpaqueFunction(function=launch_setup)]
        )