import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drones = ['drone1']

    print_controller_path = get_package_share_directory('print_controller')

    map_path = os.path.join(print_controller_path, 'map', 'output_fiducial_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gcode_path',
            default_value=os.path.join(print_controller_path, 'gcodes', 'MediumCube.gcode'),
            description='Path to the G-code file'
        ),
        
        Node(package='fiducial_vlam', executable='vmap_main', output='screen',
             name='vmap_main', parameters=[{
                'publish_tfs': 1,
                'marker_length': 0.092,
                'marker_map_load_full_filename': '/home/ws/src/src/Aerial-Additive-Manufacturing/print_controller/map/output_fiducial_map.yaml',
                'make_not_use_map': 0}]),


        Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace=drones[0]),    
        Node(package='print_controller', executable='PrintControllerSingle', output='screen'),
        Node(package='print_controller', executable='PrintVisualizationSingle', output='screen'),
        Node(package='gcode_to_path', executable='PathFromGcode', output='screen',
             parameters=[{'gcode_path': '/home/ws/src/src/Aerial-Additive-Manufacturing/print_controller/gcodes/HollowCube.gcode'}]),

        *[
            Node(package='fiducial_vlam', executable='vloc_main', output='screen',
                 name='vloc_main', namespace=namespace, parameters=[{
                    'publish_tfs': 1,
                    'base_frame_id': 'base_link_' + str(idx + 1),
                    't_camera_base_z': -0.035,
                    'camera_frame_id': 'camera_link_' + str(idx + 1),
                    'publish_image_marked': 1}])
            for idx, namespace in enumerate(drones)
        ]
    ])

if __name__ == '__main__':
    generate_launch_description()

