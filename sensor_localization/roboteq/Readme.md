====================================================================================================================================================================================================================
||										Sensor Localization Package- Roboteq												  ||
====================================================================================================================================================================================================================

The <roboteq> package is originally made by its manufacturer. The main objective of this package is to open the serial port and communicate i.e. read and write data to the roboteq. The package consists of following c++ libraries and executable.

	1.<RoboteqDevice.cpp> ::This cpp library consists of all the basis of low level programming related to serial communication and initialization of communication parameters like BAUDRATE, PARITY, STOPBITS, BLOKING MODE, etc. This library is the key and the most essential part of roboteq package. It also contains function for sending a character or a string or a command to roboteq. The roboteq command can be referred from its datasheet. The <ReadAll> function reads all the data receive on the serial port which is sent by roboteq which can be encoder counts, etc.

	2.<command_subscriber_roboteq_client> :: This executable cpp file subscribes data published on the topic <”/cmd_vel”> and then sends the velocity command to <”motor_controller”> topic of the server. Then the <setspeed> callback function executes the motor command.

	3.<roboteq_client.cpp> :: This cpp file take receives 2 arguments which are commands for Left Motor Velocity and Right Motor Velocity. It then calls the <roboteq_server> which then send the velocity commands to roboteq. The command is given to <”motor_controller”> <topic> of <”motor_controller_server”> <node>.

	4.<roboteq_encoder_client.cpp> :: This cpp file is a client which when executed prints the Motor Left and Right Velocities. It call the <server_client> which then calls <getspeed> call back function to get the current motor velocities.

	5.<roboteq_server.cpp> :: This server cpp main function is to respond to clients demand which can be either to GET and SET motor velocity. It has two callback functions <getspeed> and <setspeed>.   It has get data sent on two topics. 
		a) <”motor_controller”> for setting the motor speed.
		b)<”motor_speed”> for getting the motor speed.


Publishers/Subscribers:

  Publishers:
      1.<cmd_vel>	   : publishes the command sent to roboteq
  
  Server/Client:
      1.<motor_controller> :set the motor speed 
      2.<motor_speed>	   :get the motor speed



The encoder data is converted to motor velocity keeping the time track and the heading direction and orientation of robot is calculated.The velocity is subscribed by the <ekf_robot_localization> package and to the kalman filter algorithm.The <plot_localization> package plots the raw data values and the filtered ekf data in the GUI using <gnuplot>.

====================================================================================================================================================================================================================
====================================================================================================================================================================================================================


