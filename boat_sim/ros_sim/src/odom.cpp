#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/GetModelState.h>


class PosePub
{
public:
    PosePub();
    void start();
		void OnImuMsg(const sensor_msgs::Imu::ConstPtr& msg);
private:
	ros::NodeHandle n;
	ros::Publisher pose_pub;
	tf::TransformBroadcaster odom_broadcaster;
	ros::ServiceClient modelService;
	ros::Subscriber imuSub;
};

PosePub::PosePub()
{
  pose_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	modelService = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	imuSub = n.subscribe("imu",1000,&PosePub::OnImuMsg,this);
}

void PosePub::start()
{
    ros::spin();
}

void PosePub::OnImuMsg(const sensor_msgs::Imu::ConstPtr& msg)
{
	ROS_INFO("IMU: %f", msg->orientation.x);
	
  gazebo_msgs::GetModelState srv;
	//TODO: "imarc" should come from parameter server for future configuration	
  srv.request.model_name = "imarc";
  if (modelService.call(srv))
  {
		geometry_msgs::Point pose = srv.response.pose.position;
    ROS_INFO("Location: %f, %f, %f", pose.x, pose.y, pose.z);

	
		//publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = msg->header.stamp;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "body";

		odom_trans.transform.translation.x = pose.x;
		odom_trans.transform.translation.y = pose.y;
		odom_trans.transform.translation.z = pose.z;
		odom_trans.transform.rotation = msg->orientation;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_publisher");
	PosePub node;
	node.start();

  return 0;
}





