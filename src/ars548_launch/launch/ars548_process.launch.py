import os  
from launch import LaunchDescription  # launch文件的描述类
from launch_ros.actions import Node   # 节点启动的描述类
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法
from launch.actions import ExecuteProcess
from datetime import datetime  


def generate_launch_description():  
    # 获取当前时间并格式化为字符串  
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')  
    bag_file_name = f'my_bag_file_{timestamp}'  # 包文件名包含时间戳  

    rviz_config = os.path.join(          # 找到配置文件的完整路径
      get_package_share_directory('ars548_launch'),
      'rviz',
      'ros2_rviz.rviz'
      )
    return LaunchDescription([ 
        Node(  
            package='ars548_process',  # 替换为您的包名  
            executable='info_convert_node',  # 替换为您的节点可执行文件  
            name='info_convert_node',  # 可选的节点名称  
            output='screen',  # 输出到终端  
            parameters=[{'param_name': 'param_value'}]  # 可选的参数  
        ), 
        Node(  
            package='ars548_process',  # 替换为您的包名  
            executable='ars548_process_node',  # 替换为您的节点可执行文件  
            name='ars548_process_node',  # 可选的节点名称  
            output='screen',  # 输出到终端  
            parameters=[{'param_name': 'param_value'}]  # 可选的参数  
        ), 
        #ros2 run usb_cam usb_cam_node_exe
        Node(  
            package='usb_cam',  # 替换为您的包名  
            executable='usb_cam_node_exe',  # 替换为您的节点可执行文件  
            name='usb_cam_node_exe',  # 可选的节点名称  
            output='screen',  # 输出到终端  
            parameters=[{'param_name': 'param_value'}]  # 可选的参数  
        ), 
        Node(                             # 配置一个节点的启动
            package='rviz2',               # 节点所在的功能包
            executable='rviz2',            # 节点的可执行文件名
            name='rviz2',                  # 对节点重新命名
            arguments=['-d', rviz_config]  # 加载命令行参数
        ),
        # 添加更多节点，如果需要的话
        # 启动 rosbag 录制  
        #排除某些主题：如果希望记录大部分主题但排除某些主题，可以使用 -x 选项 '-x (.*)/compressed(.*)'
        ExecuteProcess(  
            cmd=['ros2', 'bag', 'record','-o', bag_file_name, '/ars548_process/detection_list', '/image_raw'],  
            output='screen',  
        ),   
    ])