#include "ros/ros.h"
#include <std_msgs/String.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_input_pub");
  ros::NodeHandle n;
  ros::Publisher trajectory_publisher = n.advertise<std_msgs::String>("keyboard_input_topic", 1000);

  //ros::Rate loop_rate(10);
  ros::Rate loop_rate(0.1);
  
  int count = 0;

  while (ros::ok())
  {
  
    std_msgs::String keyboard_input;
    std::string input;
    
    //std::cout << "Enter \"opposite direction\" to change the robot trajectory (Other input changes trajectory back to original)" << std::endl;
    std::cout << "Enter \"opposite direction\" to change the robot trajectory to opposite, \"original direction\" to change trajectory back to original, \"stop\" to stop robot or \"continue\" to continue after stopping" << std::endl;
    std::getline(std::cin, input);
    
    for (char &c : input)
    {
    	c = std::tolower(c);
    }
    
    if (input == "opposite direction")
    {
    	keyboard_input.data = "opposite";
    }
    else if (input == "original direction")
    {
    	keyboard_input.data = "original";
    }
    else if (input == "stop")
    {
    	keyboard_input.data = "stop";
    }
    else if (input == "continue")
    {
    	keyboard_input.data = "continue";
    }
    else
    {
    	keyboard_input.data = "unknown";
    }

    ROS_INFO("Publisher publishing!");
    trajectory_publisher.publish(keyboard_input);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
