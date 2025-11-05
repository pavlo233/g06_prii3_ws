1/TURTLESIM

REQUISITOS

----tener ros instalado

----clonar repositorio

--entrar al repo g06_prii3_ws

source /opt/ros/foxy/setup.bash

colcon build

source install/setup.bash

ros2 launch g06_prii3_turtlesim turtle.launch.py


-abrir otro terminal para usar el service

source /opt/ros/foxy/setup.bash


-tres opciones reiniciar pausar y reanudar


 ros2 topic pub /turtle_command std_msgs/msg/String "data: 'reset'" -1
 
 ros2 topic pub /turtle_command std_msgs/msg/String "data: 'pause'" -1
 
 ros2 topic pub /turtle_command std_msgs/msg/String "data: 'resume'" -1
__________________________________________________________________________________________________________________________________________________________________
__________________________________________________________________________________________________________________________________________________________________
2/JETBOT

REQUISITOS

----tener ros instalado

----clonar repositorio

--entrar al repo g06_prii3_ws

source /opt/ros/foxy/setup.bash

colcon build 

source install/setup.bash


-Entrar al robot mediante:

$ ssh -X jetbot@IP_del_robot

$ ros2 launch jetbot_pro_ros2 jetbot.py #Inicia el nodo de control del robot


--tres opciones

ros2 launch mov6 launch_jetbot.py #para que haga el 6

ros2 launch mov6 launch_jetbot_lidar.py #seis con marcha paro por deteccion lidar

ros2 launch mov6 launch_jetbot_lidar_avoidance.py #seis con evitamiento de objetos (aun no perfeccionado)


-abrir otro terminal para usar el service

source /opt/ros/foxy/setup.bash



-tres opciones reiniciar pausar y reanudar

ros2 topic pub /turtle_command std_msgs/String "data: 'reset'" -1

ros2 topic pub /turtle_command std_msgs/String "data: 'pause'" -1

ros2 topic pub /turtle_command std_msgs/String "data: 'resume'" -1

__________________________________________________________________________________________________________________________________________________________________

__________________________________________________________________________________________________________________________________________________________________

3/TURTLEBOT

REQUISITOS

----tener ros instalado

----clonar repositorio

--entrar al repo g06_prii3_ws

source /opt/ros/foxy/setup.bash

colcon build

source install/setup.bash

--abrir un terminal y poner estos comandos para abrir el si,ulador gazebo

$export TURTLEBOT3_MODEL=burger

$ros2 launch turtlebot3_gazebo empty_world.launch.py



--tres opciones en otra terminal

ros2 launch g06_prii3_move_turtlebot launch_turtlebot.py #para que haga el 6

ros2 launch g06_prii3_move_turtlebot launch_turtlebot_lidar.py #seis con marcha paro por deteccion lidar

ros2 launch g06_prii3_move_turtlebot launch_turtlebot_lidar_avoidance.py #seis con evitamiento de objetos



--abrir otro terminal para usar el service

source /opt/ros/foxy/setup.bash


--tres opciones reiniciar pausar y reanudar

ros2 topic pub /turtle_command std_msgs/String "data: 'reset'" -1

ros2 topic pub /turtle_command std_msgs/String "data: 'pause'" -1

ros2 topic pub /turtle_command std_msgs/String "data: 'resume'" -1

