from launch import LaunchDescription
import launch_ros.actions
from os import path


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package ='hesai_lidar',
            namespace ='hesai',
            executable ='hesai_lidar_node',
            name ='hesai_node',
            output ="screen",
            parameters=[
                {"pcap_file": ""},
                {"server_ip"  : "192.168.1.100"},
                {"lidar_recv_port"  : 2368},
                {"gps_port"  : 10110},
                {"start_angle"  : 0.0},
                {"lidar_type"  : "PandarXT-32"},
                {"frame_id"  : "PandarXT-32"},
                {"pcldata_type"  : 0},
                {"publish_type"  : "both"},
                {"timestamp_type"  : ""},
                {"data_type"  : ""},
                {"lidar_correction_file"  : path.dirname(path.abspath(__file__))+"/../config/PandarXT-32.csv"},
                {"multicast_ip"  : ""},
                {"coordinate_correction_flag"  : False},
                {"fixed_frame"  : ""},
                {"target_frame_frame"  : ""}
            ]
        )
    ])



