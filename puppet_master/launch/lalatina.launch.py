#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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
              respawn_delay=2.0,
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
              parameters=[amcl_file])
  
  #####################################################################################################################################################
  #----------------------------------------------------------------------controller-------------------------------------------------------------------#
  #####################################################################################################################################################

  controller_file = os.path.join(get_package_share_directory('puppet_master'), 'params', 'nav2_params_controller.yaml')

  remapping = [('/tf', 'tf'),('/tf_static', 'tf_static')]


  controller_node = Node(
              package='nav2_controller',
              executable='controller_server',
              name='controller_server',
              respawn='True',
              respawn_delay=2.0,
              parameters=[controller_file],
              remappings=remapping + [('cmd_vel', 'cmd_vel_nav')])
  
  smoother_node = Node(
              package='nav2_smoother',
              executable='smoother_server',
              name='smoother_server',
              parameters=[controller_file],
              remappings=remapping)
  
  planner_node = Node(
              package='nav2_planner',
              executable='planner_server',
              name='planner_server',
              parameters=[controller_file],
              remappings=remapping)
  
  behaviors_node = Node(
              package='nav2_behaviors',
              executable='behavior_server',
              name='behavior_server',
              parameters=[controller_file],
              remappings=remapping)
  
  navigator = Node(
              package='nav2_bt_navigator',
              executable='bt_navigator',
              name='bt_navigator',
              parameters=[controller_file],
              remappings=remapping)
  
  waypoint_follower_node = Node(
              package='nav2_waypoint_follower',
              executable='waypoint_follower',
              name='waypoint_follower',
              parameters=[controller_file],
              remappings=remapping)
  
  velocity_smoother_node = Node(
              package='nav2_velocity_smoother',
              executable='velocity_smoother',
              name='velocity_smoother',
              parameters=[controller_file],
              remappings=remapping + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])

  #####################################################################################################################################################
  #----------------------------------------------------------------------lifecycle--------------------------------------------------------------------#
  #####################################################################################################################################################

  lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl', 'controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother']}])   
    
  
  # Define LaunchDescription variable
  ld = LaunchDescription()
  
  ld.add_action(rviz_node)
  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)
  ld.add_action(tf_footprint_base_node)
  ld.add_action(tracer_base_node)
  ld.add_action(lifecycle_node)
  ld.add_action(map_server_node)
  ld.add_action(amcl_node)
  ld.add_action(controller_node)
  ld.add_action(smoother_node)
  ld.add_action(planner_node)
  ld.add_action(behaviors_node)
  ld.add_action(navigator)
  ld.add_action(waypoint_follower_node)
  ld.add_action(velocity_smoother_node)

  return ld
  
  #ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/lalatina/ros2_ws/src/puppet_master/maps/my_map.yaml}"
