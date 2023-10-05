#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "trajectory_publisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher trajectory_publisher = n.advertise<std_msgs::Float64MultiArray>("trajectory_topic", 1000);

  //ros::Rate loop_rate(10);
  ros::Rate loop_rate(0.1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  double x=0.1;
  double y=0.1;
  double z= 0.5; //??
  double rot1=0;
  double rot2=0;
  double rot3=0;
  
  std::vector<double> point1;
  std::vector<double> point2;
  std::vector<double> point3;
  std::vector<double> point4;
  //std::vector<double> point5;
  
  std_msgs::Float64MultiArray trajectory_msg;
  
  for (int i = 0; i < 4; i++)
  {
  	trajectory_msg.data.push_back(x);
  	trajectory_msg.data.push_back(y);
  	trajectory_msg.data.push_back(z);
  	trajectory_msg.data.push_back(rot1);
  	trajectory_msg.data.push_back(rot2);
  	trajectory_msg.data.push_back(rot3);
  	x+=0.1;
  	y+=0.1;		
  }
  
  ROS_INFO("Publisher publishing!");
  trajectory_publisher.publish(trajectory_msg);
  ros::spinOnce();
  
  //while (ros::ok())
  //{
  
    //std_msgs::Float64MultiArray trajectory_msg;
  /* Test 1:  
    if (count <= 500)
    {
    	//x += 0.1;

    }
    else if (count > 500 && count <=1000)
    {
    	//x=5;
    	//y+=0.1;
    	x+=0.0001;
    }
    else if (count>1000 && count <= 1500)
    {
    	//x-=0.1;
    	y+=0.0001;
    }
    else if (count>1500 && count <= 2000)
    {
    	//y-=0.1;
    	x-=0.0001;
    }
    else if (count>2000 && count <= 2500)
    {
    	//y-=0.1;
    	y-=0.0001;
    }*/
    
    /* Test 2: if (count == 0)
    {
    	x = 0.3;
    }
    else if (count==1)
    {
    	y=0.3;
    }
    else if (count==2)
    {
    	x=0.1;
    }
    else if (count==3)
    {
    	y=0.1;
    }
    
    trajectory_msg.data.push_back(x);
    trajectory_msg.data.push_back(y);
    trajectory_msg.data.push_back(z);
    trajectory_msg.data.push_back(rot1);
    trajectory_msg.data.push_back(rot2);
    trajectory_msg.data.push_back(rot3);
    
    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    ROS_INFO("Publisher publishing!");
    

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //trajectory_publisher.publish(trajectory_msg);

    //ros::spinOnce();

    //loop_rate.sleep();
    //++count;
  //}


  return 0;
}
