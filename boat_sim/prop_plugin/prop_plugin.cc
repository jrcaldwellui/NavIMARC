#ifndef _PROP_PLUGIN_HH_
#define _PROP_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <stdio.h>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"


namespace gazebo
{
	class PropPlugin : public ModelPlugin
	{
		public: PropPlugin(){}
		public:	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			std::cerr<<"\nThe prop plugin is attached to "<< _model->GetName() << "\n";
			propForce = 100;

			if(_sdf->HasElement("force"))
				propForce = _sdf->Get<double>("force");
			
			this->leftPropOn =false;
			this->rightPropOn = false;
			this->currentLeftPropOn = false;
			this->currentRightPropOn = false;
			this->boatBody = _model->GetLink("body"); 
			if(this->boatBody == NULL)
			{
				std::cerr << "_model->GetLink(\"body\") did not find link named body, be sure the model with plugin attached has link body\n";
				return;
			}
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PropPlugin::OnUpdate,this,_1));


			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "keyboard_client",
				  ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("keyboard_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			  ros::SubscribeOptions::create<std_msgs::String>(
				  "/input",
				  1,
				  boost::bind(&PropPlugin::OnRosMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&PropPlugin::QueueThread, this));
		}

		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the velocity
		/// of the Velodyne.
		public: void OnRosMsg(const std_msgs::StringConstPtr &_msg)
		{
		  
			if(_msg->data.compare("119")==0)
			{
				std::cerr << "fwd\n";
				leftPropOn = true;
				rightPropOn = true;
			}
			else if(_msg->data.compare("97")==0 )
			{
				std::cerr << "left\n";
				leftPropOn = true;
				rightPropOn = false;
			}
			else if(_msg->data.compare("100")==0)
			{
				std::cerr << "right\n";
				leftPropOn = false;
				rightPropOn = true;
			}else if(_msg->data.compare("115")==0)
			{
				std::cerr << "stop\n";
				leftPropOn = false;
				rightPropOn = false;
			}else if(_msg->data.compare("112")==0)
			{
				std::cerr <<"----------------------\n";
				std::cerr << "Force: " << this->boatBody->GetWorldForce() << "\n" ;
				std::cerr << "Vel of Cog: " << this->boatBody->GetWorldCoGLinearVel() <<"\n";
				std::cerr << "Angular Vel: " << this->boatBody->GetWorldAngularVel() <<"\n";
				std::cerr <<"----------------------\n";
			}


		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
		  static const double timeout = 0.01;
		  while (this->rosNode->ok())
		  {
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		  }
		}

		public: void OnUpdate(const common::UpdateInfo &)
		{
			//std::cerr << this->boatBody->GetWorldForce() << "\n" << "left: " << leftPropOn << " right: " << rightPropOn << "\n" << currentLeftPropOn << currentRightPropOn << "\n";
			if(currentLeftPropOn != leftPropOn || currentRightPropOn != rightPropOn)
			{
				std::cerr << this->boatBody->GetWorldForce() << "\n" << "left: " << leftPropOn << " right: " << rightPropOn << "\n" << currentLeftPropOn << currentRightPropOn << "\n";
				this->boatBody->SetForce( math::Vector3(0,0,0) );
				currentLeftPropOn = leftPropOn;
				currentRightPropOn = rightPropOn;
			}

			if(leftPropOn)
			{
				this->boatBody->AddForceAtRelativePosition(math::Vector3(-propForce,0,0),math::Vector3(-1.219*0.5,-1/*-0.737*0.33*/,0));
			}	
			if(rightPropOn)
			{
				this->boatBody->AddForceAtRelativePosition(math::Vector3(-propForce,0,0),math::Vector3(-1.219*0.5,1/*0.737*0.33*/,0));
			}	
			
		
			
			


	//std::cerr<<"UPDATE event\n";
			

		}
		
		private: event::ConnectionPtr updateConnection;
		private: physics::LinkPtr boatBody;

		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

		private: bool leftPropOn;
		private: bool rightPropOn;

		private: bool currentLeftPropOn;
		private: bool currentRightPropOn;
		private: double propForce;	
	};

	

	GZ_REGISTER_MODEL_PLUGIN(PropPlugin)
}

#endif
