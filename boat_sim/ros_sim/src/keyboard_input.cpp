#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <termios.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("input", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
	int in = getch();
	
	if(in)
	{	
		std_msgs::String msg;
		std::stringstream ss;
		ss << in;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

	  
		chatter_pub.publish(msg);
	}

  
	ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
