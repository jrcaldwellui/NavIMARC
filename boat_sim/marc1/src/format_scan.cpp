#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


class FormatScan
{
public:
    FormatScan();
    void start();
    void onScanMsg(const sensor_msgs::LaserScan::ConstPtr& msg);
    int inline safeIterAdv(std::vector<float>::const_iterator& it, std::vector<float>::const_iterator& end, size_t n);
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

//https://stackoverflow.com/questions/3856416
int inline FormatScan::safeIterAdv(std::vector<float>::const_iterator& it, std::vector<float>::const_iterator& end, size_t n)
{
	size_t i= 0;
	for(; i !=n && it!=end;++i,++it);
	return n-i;
}

void FormatScan::onScanMsg(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Each msg->rangs contains two rotations worth of ranges
	//std::cout << "go time";
	//number of scans per two rotations
	int desiredNumOfScans = 800;
	size_t inc = msg->ranges.size()/desiredNumOfScans;

	std::vector<float> newRanges;
	newRanges.reserve(desiredNumOfScans);

	std::vector<float>::const_iterator end = msg->ranges.end();	
	for(std::vector<float>::const_iterator it = msg->ranges.begin(); it != end;)
	{
		std::cout << *it;
		newRanges.push_back(*it);
		safeIterAdv(it,end,inc);
	}
	


	int size = newRanges.size();
	std::vector<float> intensities(size,0.0);
	sensor_msgs::LaserScan newMsg;

	//copy msg to newMsg, someone better at c++ plz do this cleaner
	newMsg.header = msg->header;
	newMsg.angle_min = msg->angle_min;
	newMsg.angle_max = msg->angle_max;
	newMsg.angle_increment = 6.28 / (float)newRanges.size();
	newMsg.time_increment = msg->time_increment;
	newMsg.scan_time = 0.0;	
	newMsg.range_min = 0.009999;
	newMsg.range_max = msg->range_max;
	newMsg.ranges = newRanges;
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


