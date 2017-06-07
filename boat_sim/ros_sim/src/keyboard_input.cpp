#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

#include <sstream>
#include <termios.h>

//http://www.cplusplus.com/forum/unices/18395/
//Reads and returns char without pressing enter
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

// Returns val limited to range 0 to 100
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

//Modifies lDutyCycle and rDutyCycle based on value of in
//
// w: both inc
// s: both dec
//
// q: left inc
// a: left dec
//
// e: right inc
// d: right dec
void ProcessInput(char in,int &lDutyCycle,int &rDutyCycle, int stepSize = 20)
{
		if( in == 'w' )
		{
			lDutyCycle += stepSize;
			rDutyCycle += stepSize;
		}
		else if(in == 's' )
		{
			lDutyCycle -= stepSize;
			rDutyCycle -= stepSize;
		}
		else if(in == 'q' )
		{
			lDutyCycle += stepSize;
		}
		else if(in == 'e' )
		{
			rDutyCycle += stepSize;
		}
		else if(in == 'a' )
		{
			lDutyCycle -= stepSize;
		}
		else if(in == 'd' )
		{
			rDutyCycle -= stepSize;
		}
}


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
		ProcessInput(in,lDutyCycle,rDutyCycle);

		lDutyCycle = ClampValue(lDutyCycle);
		rDutyCycle = ClampValue(rDutyCycle);		
		
		std_msgs::Int8 lMsg;
		std_msgs::Int8 rMsg;
		lMsg.data = lDutyCycle;
		rMsg.data = rDutyCycle;

	  	lPub.publish(lMsg);
		rPub.publish(rMsg);
	}
  
	ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
