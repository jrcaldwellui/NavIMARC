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
#include "std_msgs/Int8.h"
#include <math.h>


namespace gazebo
{
	/*
	* Gazebo plugin that creates two forces simplistically simulating a left and right 
	* propeller

	* subscribes to ROS topics: 	
	* 	/l_duty_cycle int8 values 0 to 100 sets left propeller duty cycle in %
	* 	/r_duty_cycle int8 values 0 to 100 sets right propeller duty cycle in % 
	* 	/input string general keyboard input
	*/
	class PropPlugin : public ModelPlugin
	{
		public: PropPlugin(){}


		//Called by Gazebo when plugin is inserted into simulation
		public:	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			std::cerr<<"\nThe prop plugin is attached to "<< _model->GetName() << "\n";
			this->maxPropForce = 50;
			this->lDutyCycle = 0.0;
			this->rDutyCycle = 0.0;

			//Check for force xml tag in sdf file
			if(_sdf->HasElement("force"))
				this->maxPropForce = _sdf->Get<float>("force");
			
			this->boatBody = _model->GetLink("body"); 

			if(this->boatBody == NULL)
			{
				std::cerr << "_model->GetLink(\"body\") did not find link named body, be sure the model with plugin attached has link body\n";
				return;
			}
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PropPlugin::OnUpdate,this,_1));


			// Initialize ros, if it has not already been initialized.
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

			// Create input topic, and subscribe to it.
			ros::SubscribeOptions so =
			  ros::SubscribeOptions::create<std_msgs::String>(
				  "/input",
				  1,
				  boost::bind(&PropPlugin::OnKeyMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);

			this->keyboardSub = this->rosNode->subscribe(so);

			// Create /l_duty_cycle topic, and subscribe to it.
			ros::SubscribeOptions so1 =
			  ros::SubscribeOptions::create<std_msgs::Int8>(
				  "/l_duty_cycle",
				  1,
				  boost::bind(&PropPlugin::OnLPropMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);

			this->lPropSub = this->rosNode->subscribe(so1);

			// Create /r_duty_cycle topic, and subscribe to it.
			ros::SubscribeOptions so2 =
			  ros::SubscribeOptions::create<std_msgs::Int8>(
				  "/r_duty_cycle",
				  1,
				  boost::bind(&PropPlugin::OnRPropMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);

			this->rPropSub = this->rosNode->subscribe(so2);


			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&PropPlugin::QueueThread, this));
		}

		//Sets left propeller duty cycle when message is recieved from topic /l_duty_cycle
		public: void OnLPropMsg(const std_msgs::Int8ConstPtr &_msg)
		{
			this->lDutyCycle = ClampDutyCycle(_msg->data);
		}

		//Sets right propeller duty cycle when message is recieved from topic /r_duty_cycle
		public: void OnRPropMsg(const std_msgs::Int8ConstPtr &_msg)
		{
			this->rDutyCycle = ClampDutyCycle(_msg->data);	
		}

		// dutyCycle - int expected to be from 0 to 100
		// returns - float converts dutyCycle as percentage to decimal reprensentation range (0.0 to 1.0), 
		// 	if dutyCycle falls outside the range of 0 to 100, 0 or 1 will be returned respectivly 
		private: float ClampDutyCycle(int dutyCycle)	
		{
			float clampedDC = (float)dutyCycle;
			if(clampedDC > 100)
			{
				clampedDC = 100;
			}else if(clampedDC < 0)
			{
				clampedDC = 0;
			}
			return clampedDC/100.0;
		}

		// Handles an incoming keyboard messages from ROS
		public: void OnKeyMsg(const std_msgs::StringConstPtr &_msg)
		{
			//Prints info about boat model, key: p
			if(_msg->data.compare("112")==0)
			{
				std::cerr <<"----------------------\n";
				std::cerr << "Rotation: " << this->boatBody->GetWorldPose().rot.GetAsEuler() * 180.0 / 3.14 << "\n";
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

		//Called every Gazebo 
		public: void OnUpdate(const common::UpdateInfo &)
		{

			float boatAngle = this->boatBody->GetWorldPose().rot.GetAsEuler().z;

			//std::cerr << force << "\n";
			if(lDutyCycle > 0)
			{
				float forceMag = this->lDutyCycle * this->maxPropForce;
				math::Vector3 force = math::Vector3(forceMag * cos(boatAngle), forceMag * sin(boatAngle),0 );
				this->boatBody->AddForceAtRelativePosition(force,math::Vector3(-1.219*0.5,-1/*-0.737*0.33*/,0));
				
				
			}	
			if(rDutyCycle > 0)
			{
				float forceMag = this->rDutyCycle * this->maxPropForce;
				math::Vector3 force = math::Vector3(forceMag * cos(boatAngle), forceMag * sin(boatAngle),0 );
				this->boatBody->AddForceAtRelativePosition(force,math::Vector3(-1.219*0.5,1/*0.737*0.33*/,0));
			}	
			
			

		}
		
		private: event::ConnectionPtr updateConnection;

		private: physics::LinkPtr boatBody;


		// A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		// ROS subscribers
		private: ros::Subscriber keyboardSub;
		private: ros::Subscriber rPropSub;
		private: ros::Subscriber lPropSub;

		// A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
		
		// Current duty cycle
		private: float rDutyCycle;
		private: float lDutyCycle;

		//Maximum propeller force, can be set in urdf
		private: float maxPropForce;	
	};

	

	GZ_REGISTER_MODEL_PLUGIN(PropPlugin)
}

#endif
