from launch import LaunchDescription, substitutions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import FindExecutable

number_of_bots = 30

def generate_launch_description():
    ld = LaunchDescription()

    artbotsim_node = Node(
        package="artbotsim",
        executable="artbotsim_node",
    )
    ld.add_action(artbotsim_node)

    move_bot = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " service call ",
            "/artist1/teleport_absolute ",
            "artbotsim/srv/TeleportAbsolute ",
            TextSubstitution(text="\"{'x': 2, 'y': 10, 'theta': 0}\"")
        ]],
        shell=True
    )
    ld.add_action(move_bot)

    x, y = 4, 10

    for i in range(1, number_of_bots):
        spawn_bot = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/spawn ",
                "artbotsim/srv/Spawn ",
                "\"{'x': "+str(x)+", 'y': "+str(y)+", 'theta': 0, 'name': ''}\""
            ]],
            shell=True
        )
        x = x + 2
        if(x > 20):
            x = 2
            y = y - 2
        ld.add_action(spawn_bot)

    return ld