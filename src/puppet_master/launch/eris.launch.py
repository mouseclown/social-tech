#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
  
  #####################################################################################################################################################
  #---------------------------------------------------------------------LiDAR-------------------------------------------------------------------------#
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
  #--------------------------------------------------------------------tracer-------------------------------------------------------------------------#
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
  #----------------------------------------------------------------------map--------------------------------------------------------------------------#
  #####################################################################################################################################################
  
  map_file = os.path.join(get_package_share_directory('puppet_master'), 'maps', 'aula_robotica.yaml')

  map_server_node = Node(
              package='nav2_map_server',
              executable='map_server',
              name='map_server',
              output='screen',
              parameters=[
                  {'yaml_filename': map_file},#custom map
                  {'topic_name': 'map'},
                  {'frame_id': 'map'}
                  ]
            )
              
  #####################################################################################################################################################
  #----------------------------------------------------------------------amcl-------------------------------------------------------------------------#
  #####################################################################################################################################################

  amcl_file = os.path.join(get_package_share_directory('puppet_master'), 'params', 'nav2_params_amcl.yaml')

  amcl_node = Node(
              package='nav2_amcl',
              executable='amcl',
              name='amcl',
              output='screen',
              parameters=[amcl_file]#custom config file
            )
              
  lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}])   
  
  #####################################################################################################################################################
  #----------------------------------------------------------------------navigation-------------------------------------------------------------------#
  #####################################################################################################################################################

  controller_file = os.path.join(get_package_share_directory('puppet_master'), 'params', 'nav2_params_navigation.yaml')

  nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': controller_file,
                              'container_name': 'nav2_container'}.items())
  
  #####################################################################################################################################################
  #----------------------------------------------------------------------3D-model---------------------------------------------------------------------#
  #####################################################################################################################################################

  model_file = os.path.join(get_package_share_directory('puppet_master'), 'urdf', 'tracer_v1.urdf.xacro')
  
  robot_state_publisher_node = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": model_file
                    }
                ]
            )
  
  # Define LaunchDescription variable
  ld = LaunchDescription()
  
  ld.add_action(rviz_node)
  ld.add_action(lifecycle_node)
  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)
  ld.add_action(tf_footprint_base_node)
  ld.add_action(tracer_base_node)
  ld.add_action(map_server_node)
  ld.add_action(amcl_node)
  ld.add_action(nav_launch)
  ld.add_action(robot_state_publisher_node)

  return ld
  
  #ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/lalatina/ros2_ws/src/puppet_master/maps/my_map.yaml}"
