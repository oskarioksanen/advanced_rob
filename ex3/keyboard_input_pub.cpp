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
    std::cout << "\n";
    std::cout << "Enter \"opposite direction\" or \"o\" to change the robot trajectory (Other input changes trajectory back to original):" << std::endl;
    std::getline(std::cin, input);
    
    for (char &c : input)
    {
    	c = std::tolower(c);
    }
    
    if (input == "opposite direction" or "o")
    {
    	keyboard_input.data = "opposite";
    }
    else
    {
    	keyboard_input.data = "original";
    }

    ROS_INFO("Publisher publishing!");
    std::cout << "\n";
    trajectory_publisher.publish(keyboard_input);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
