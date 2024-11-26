#!/usr/bin/env python3
import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():

  #####################################################################################################################################################
  #----------------------------------------------------------------------LiDAR------------------------------------------------------------------------#
  #####################################################################################################################################################

  ldlidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD06',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 0.0},
        {'angle_crop_max': 180.0}#angle crop
      ]
  )

  # base_link to base_laser tf NODE
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld06',
    arguments=['0.34','0','0.1','-1.57','0','0','base_link','base_laser']#transform of LiDAR position
  )
  
  
  #####################################################################################################################################################
  #---------------------------------------------------------------------tracer------------------------------------------------------------------------#
  #####################################################################################################################################################

  tracer_base_node = Node(
      package='tracer_base',
      executable='tracer_base_node',
      output='screen',
      emulate_tty=True,
      parameters=[
              {'use_sim_time': False},
      	      {'port_name': 'can0'},
              {'odom_frame': 'odom'},
      	      {'base_frame': 'base_link'},
      	      {'odom_topic_name': 'odom'},
      	      {'is_tracer_mini': False},
      	      {'simulated_robot': False},
      	      {'control_rate': 50}
      ]
  )
  
  tf_footprint_base_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='tf_footprint_base',
    arguments=['0','0','0','0','0','0','base_link','base_footprint']#transform to base_footprint
  )
  
  
  #####################################################################################################################################################
  #----------------------------------------------------------------------rviz-------------------------------------------------------------------------#
  #####################################################################################################################################################

  rviz_file = os.path.join(get_package_share_directory('puppet_master'), 'config', 'arcanletia.rviz')
        
  rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file]#custom config file
        )


  #####################################################################################################################################################
  #----------------------------------------------------------------------slam-------------------------------------------------------------------------#
  #####################################################################################################################################################

  use_sim_time = LaunchConfiguration('use_sim_time')
  slam_params_file = LaunchConfiguration('slam_params_file')

  declare_use_sim_time_argument = DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use simulation/Gazebo clock')
  declare_slam_params_file_cmd = DeclareLaunchArgument(
      'slam_params_file',
      default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                 'config', 'mapper_params_online_async.yaml'),
      description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

  start_async_slam_toolbox_node = Node(
      parameters=[
        slam_params_file,
        {'use_sim_time': use_sim_time}
      ],
      package='slam_toolbox',
      executable='async_slam_toolbox_node',
      name='slam_toolbox',
      output='screen')
  
  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)
  ld.add_action(tf_footprint_base_node)
  ld.add_action(tracer_base_node)
  ld.add_action(rviz_node)
  ld.add_action(declare_use_sim_time_argument)
  ld.add_action(declare_slam_params_file_cmd)
  ld.add_action(start_async_slam_toolbox_node)

  return ld
