#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


class FormatScan
{
public:
    FormatScan();
    void start();
	void onScanMsg(const sensor_msgs::LaserScan::ConstPtr& msg);
private:
	ros::NodeHandle n;
	ros::Publisher newScanPub;
	ros::Subscriber scanSub;
};

FormatScan::FormatScan()
{
	newScanPub = n.advertise<sensor_msgs::LaserScan>("reformatted_scan", 10);
	scanSub = n.subscribe("scan",10,&FormatScan::onScanMsg,this);
}

void FormatScan::start()
{
    ros::spin();
}

void FormatScan::onScanMsg(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int size = msg->ranges.size();
	std::vector<float> intensities(size,0.0);
	sensor_msgs::LaserScan newMsg;

	//copy msg to newMsg, someone better at c++ plz do this cleaner
	newMsg.header = msg->header;
	newMsg.angle_min = msg->angle_min;
	newMsg.angle_max = msg->angle_max;
	newMsg.angle_increment =msg->angle_increment;
	newMsg.time_increment = msg->time_increment;
	newMsg.scan_time = msg->scan_time;	
	newMsg.range_min = msg->range_min;
	newMsg.range_max = msg->range_max;
	newMsg.ranges = msg->ranges;
	// --------------

	newMsg.intensities =  intensities;
	newScanPub.publish(newMsg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "format_scan");
	FormatScan node;
	node.start();
	return 0;
}


