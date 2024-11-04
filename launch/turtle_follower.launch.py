from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    arduino_port = DeclareLaunchArgument(
        name="arduino_port",
        default_value="/dev/ttyACM0",
        description="The arduino port"
    )

    arduino_baudrate = DeclareLaunchArgument(
        name="arduino_baudrate",
        default_value='115200',
        description="The baudrate of the arduino"
    )

    port = LaunchConfiguration("arduino_port")
    baudrate = LaunchConfiguration("arduino_baudrate")

    give_permission_to_port = ExecuteProcess(
        cmd=['sudo', 'chmod', '666', '/dev/ttyACM0']
    )
    write_pass = ExecuteProcess(
        cmd=['George']
    )
    spawn_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', """ {x: 5.5 ,y: 5.5 ,theta: 0.0 ,name: 'turtle2'} """]
    )
    set_pen_2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/turtle2/set_pen', 'turtlesim/srv/SetPen', """ {r: 255.0, g: 150.0, b: 0, width: 3.0, 'off': 0} """]
    )
    start_turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    turtle_controller = Node(
        package="turtle_controller",
        executable="t_c_exe"
    )
    turtle_leader_controller = Node(
        package="turtle_controller",
        executable="l_t_c_exe",
        parameters=[{'port': port},
                    {'baudrate': baudrate}] # the port parameter
    )

    ld.add_action(arduino_port)
    ld.add_action(arduino_baudrate)
    ld.add_action(give_permission_to_port)
    ld.add_action(write_pass)
    ld.add_action(start_turtlesim)
    ld.add_action(spawn_service)
    ld.add_action(turtle_controller)
    ld.add_action(turtle_leader_controller)
    ld.add_action(set_pen_2)

    return ld