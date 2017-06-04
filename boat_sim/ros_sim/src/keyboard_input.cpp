#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

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

int ClampValue(int val)
{
	if(val > 100)
	{
		return 100;
	}else if(val < 0)
	{
		return 0;
	}

	return val;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("input", 1000);
  ros::Publisher lPub = n.advertise<std_msgs::Int8>("l_duty_cycle", 1000);
  ros::Publisher rPub = n.advertise<std_msgs::Int8>("r_duty_cycle", 1000);

	
	ros::Rate loop_rate(10);
	int lDutyCycle = 0;
	int rDutyCycle = 0;
  while (ros::ok())
  {
	int in = getch();


	
	if(in)
	{	
		if( in == 'w')
		{
			lDutyCycle += 20;
			rDutyCycle += 20;
		}
		else if(in == 's' )
		{
			lDutyCycle -= 20;
			rDutyCycle -= 20;
		}
		else if(in == 'q' )
		{
			lDutyCycle += 20;
		}
		else if(in == 'e' )
		{
			rDutyCycle += 20;
		}
		else if(in == 'a' )
		{
			lDutyCycle -= 20;
		}
		else if(in == 'd' )
		{
			rDutyCycle -= 20;
		}

		/*std_msgs::String msg;
		std::stringstream ss;
		ss << in;
		msg.data = ss.str();
		*/

		lDutyCycle = ClampValue(lDutyCycle);
		rDutyCycle = ClampValue(rDutyCycle);
		
		std_msgs::Int8 lMsg;
		std_msgs::Int8 rMsg;
		lMsg.data = lDutyCycle;
		rMsg.data = rDutyCycle;



		//ROS_INFO("%d %d", leftPropOn,rightPropOn);

	  	lPub.publish(lMsg);
		rPub.publish(rMsg);
	}

  
	ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
