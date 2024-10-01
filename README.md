# RAFI launch files

Este paquete de ROS contiene los archivos de lanzamiento para la ejecución de los distintas funcionalidades del robot.

## Contenido

El contenido de la carpeta `/launch` es:
- `rafi_teleop_base.launch`: Se trata del archivo de lanzamiento que lanza el controlador de la base junto con los nodos relacionados con la teleoperación. El lanzamiento de este archivo permite el movimiento de la base con el mando.
- `rafi_teleop_impedance_franka.launch`: Lanza el controlador de impedancia cartesiana del manipulador y los nodos relacionados con su teleoperación. Este archivo de lanzamiento permite modificar el `equilibrium_pose` del manipulador con el mando.
- `rafi_teleop_velocity_franka.launch`: Lanza el controlador de velocidad cartesiano del manipulador junto con los nodos relacionados con su teleoperación. Este archivo permite modificar la velocidad cartesiana del efector lineal mediante el mando.
- `rafi_impedance.launch`: Lanza el controlador de la base, el controlador de impedancia cartesiana del manipulador y los nodos relacionados con la teleoperación de ambos. Este archivo permite modificar la velocidad cartesiana de la base y la pose del efector final del manipulador desde el mando.
- `rafi_velocity.launch`: Lanza el controlador de la base, el controlador de velocidad cartesiana del manipulador junto con los nodos relacionados con la teleoperación de ambos controladores. Este archivo permite comandar velocidades cartesianas para la base y velocidades cartesianas para el efector final del manipulador desde el mando.


## Instalación

### Paquetes necesarios

Para su uso, se debe tener instalados los siguientes paquetes. Se adjunta el enlace que contiene información sobre su instalación.
- `joy_base_control`. Disponible en: https://github.com/rodri-castro/joy_base_control.git
- `joy_franka_control`. Disponible en: https://github.com/rodri-castro/joy_franka_control.git
- `joy_franka_vel_control`. Disponible en: https://github.com/rodri-castro/joy_franka_vel_control.git

### Instalación de este paquete

```bash
git clone https://github.com/rodri-castro/rafi_launch_files.git
catkin_make
```

## Uso

###  Teleoperación del controlador de la base

```bash
roslaunch rafi_launch_files rafi_teleop_base.launch
```

###  Teleoperación del controlador de impedancia cartesiano del manipulador

```bash
roslaunch rafi_launch_files rafi_teleop_impedance_franka.launch
```

###  Teleoperación del controlador de velocidad cartesiano del manipulador

```bash
roslaunch rafi_launch_files rafi_teleop_velocity_franka.launch
```

###  Teleoperación del Esquema A
El esquema A corresponde al controlador de impedancia cartesiana del manipulador ejecutandose de manera simultánea con el controlador de la base.

```bash
roslaunch rafi_launch_files rafi_impedance.launch
```

###  Teleoperación del Esquema B
El esquema B corresponde al controlador de velocidad cartesiana del manipulador ejecutandose de manera simultánea con el controlador de la base.

```bash
roslaunch franka_example_controllers move_to_start.launch robot_ip:=172.16.0.2 load_gripper:=false robot:=panda
roslaunch rafi_launch_files rafi_velocity.launch
```

