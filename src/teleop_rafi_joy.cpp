/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include <map>
#include <string>
#include <cstdlib> //system()

#include "teleop_rafi_joy.h"
#include "sensor_msgs/Joy.h"


// Definir un namespace evita conflictos de nombres con otras partes del código o blibliotecas externas
namespace teleop_rafi_joy
{

  /**
   * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
   * parameters later without breaking ABI compatibility, for robots which link TeleopRafiJoy
   * directly into base nodes.
   */
  struct TeleopRafiJoy::Impl
  {
    // Members functions
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy); // Función encargada de manejar los mensajes del joystick

    // ROS subscribers and publisher
    ros::Subscriber joy_sub;

    double Delta_t = 0.001; // Tiempo en segundo
    float reaction_t = 0.5; // Tiempo en segundo de reaccion del operador

    int enable_mov_position;    // Variable que activa el control
    int enable_mov_orientation; // Variable que activa la velocidad orientation
    int home_button;

    int a_button;
    int b_button;
    int x_button;
    int y_button;

    bool B_abierto;

  };

  /**
   * Constructs TeleopRafiJoy.
   * \param nh NodeHandle to use for setting up the publisher and subscriber.
   * \param nh_param NodeHandle to use for searching for configuration parameters.
   */
  // Constructor: Inicializa los parámetros del nodo ROS y los parámetros del joystick
  TeleopRafiJoy::TeleopRafiJoy(ros::NodeHandle *nh, ros::NodeHandle *nh_param)
  {
    pimpl_ = new Impl;
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopRafiJoy::Impl::joyCallback, pimpl_); // Cuando se recibe un mensaje llama a la función callback.

    // HEREDADO: Asignar botones
    nh_param->param<int>("enable_mov_position", pimpl_->enable_mov_position, 0); // Se obtiene el parámetro del enable_mov_position del servidor de parámetros ROS, por defecto es 0.
    nh_param->param<int>("enable_mov_orientation", pimpl_->enable_mov_orientation, -1);
    nh_param->param<int>("home_button", pimpl_->home_button, -1);

    // Nuevo mapeo
    nh_param->param<int>("a_button", pimpl_->a_button, -1);
    nh_param->param<int>("b_button", pimpl_->b_button, -1);
    nh_param->param<int>("x_button", pimpl_->x_button, -1);
    nh_param->param<int>("y_button", pimpl_->y_button, -1);

    pimpl_->B_abierto = false;

  }

  // Obtiene valores de los exes (joysticks y gatillos analógicos)
  double getVal(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_map, const std::string &fieldname)
  {
    /*
    Método que obtiene valores especificos del mensaje del joystick:
    Argumentos:
      - joy_msg: mensaje joy del cual se va a obtener la informacion
      - axis_map: mapa de ejes de control
      - fieldname: campo que se quiere obtener [x,y,z] o [x,y,z,w]
    */

    if (axis_map.find(fieldname) == axis_map.end() || joy_msg->axes.size() <= axis_map.at(fieldname))
    {
      return 0.0;
      ROS_INFO("Fallo de función getVal");
    }
    ROS_INFO("Entro");
    return joy_msg->axes[axis_map.at(fieldname)];
  }

  void TeleopRafiJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {
    ROS_INFO("Leyendo mando");

    if (joy_msg->buttons[a_button])
    {
      ROS_INFO("Boton A pulsado");
      int ret = system("terminator"); // Prueba
      ROS_INFO("Retorno: %i", ret);
      (void)ret;

      /*
        if A no se ha pulsado:
          abre terminator
          if !system(terminator) -> abierto_A=1;
        else if A abierto:
          cerrar terminator
      */

    }
    else if (joy_msg->buttons[b_button]) // Boton izquierdo
    {
      ROS_INFO("Boton B pulsado");

      if (!B_abierto){
        int ret = system("rosrun rqt_graph rqt_graph");
        if(!ret){
          B_abierto=true;
          ROS_INFO("B_abierto: %s", B_abierto ? "true" : "false");
        }
        (void)ret;
    
      } else if (B_abierto){
        int ret = system("rosnode kill rqt_graph");
        if(!ret) B_abierto=false;
        (void)ret;
      }

    }

    ros::Duration(reaction_t).sleep(); // Espera un tiempo de reaccion del operador
  }

} // namespace teleop_rafi_joy