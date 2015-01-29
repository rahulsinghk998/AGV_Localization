#include <ros/ros.h>
#include <roboteq/SetSpeed.h>
#include <roboteq/RoboteqDevice.h>

//###THIS CODE IS FOR GIVING THE COMMAND FOR SETING THE MOTOR SPEED THROUGH THE TERMINAL BY GIVING THE ARGUMENTS WHICH ARE OUR DESIRED <LEFT@ENCODER_VALUE> AND <RIGHT@ENCODER_VALUE>
//##THIS CODE IS A CLIENT WHICH CALLS THE SERVER TO SET THE MOTOR SPEED BY GIVING THE ENCODER ARGUMENTS.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller_client");

//@Rahul:I don't understand the <#3> argument I.E. why the program should quit or not set the motor speed if the argument is 3.

    if (argc != 3)
    {
      ROS_INFO("usage: motor_controller_client");
      return 1;
    }
  
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<roboteq::SetSpeed>("motor_controller");
    roboteq::SetSpeed srv;
    srv.request.left_speed = atoll(argv[1]);
    srv.request.right_speed = atoll(argv[2]);
    if (client.call(srv))
    {
      ROS_INFO("Speed target set to : %ld, %ld", (long int)srv.request.left_speed, (long int)srv.request.right_speed);
      ROS_INFO("Response : %ld", (long int)srv.response.code);
    }
    else
    {
      ROS_ERROR("Failed to call service set speed");
      ROS_INFO("Response : %ld", (long int)srv.response.code);
      return 1;
    }
    
    return 0;
  }
