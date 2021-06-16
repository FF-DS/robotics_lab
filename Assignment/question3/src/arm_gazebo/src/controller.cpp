#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "arm_gazebo/JointAngleState.h"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{	
	private: ros::Publisher publishAngleState;

	public:

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{

			if( !ros::isInitialized() ){
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "jointAnglePublisher");
			}

			// Store the pointer to the model
			this->model = _parent;

			// // intiantiate the joint controller
			this->jointController = this->model->GetJointController();

			// // set your PID values
			this->pid = common::PID(30.1, 10.01, 10.03);

			auto joint_name = "arm1_arm2_joint";

			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

			this->jointController->SetPositionPID(name, pid);

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));

			
			// ros part
			ros::NodeHandle n;
			this->publishAngleState = n.advertise<arm_gazebo::JointAngleState>("JointAngles", 1000);
			ros::Rate loop_rate(10);
			
		}

		// Called by the world update start event
	public:
		void OnUpdate()
		{
			arm_gazebo::JointAngleState angleInfo;
			std::stringstream tmp1, tmp2, tmp3, tmp4;
			
			tmp1 << "name : " << this->model->GetJoints()[0]->GetName()  << " angel : " << this->model->GetJoints()[0]->Position() * 180.0 / M_PI;
			tmp2 << "name : " << this->model->GetJoints()[1]->GetName()  << " angel : " << this->model->GetJoints()[1]->Position() * 180.0 / M_PI;
			tmp3 << "name : " << this->model->GetJoints()[2]->GetName()  << " angel : " << this->model->GetJoints()[2]->Position() * 180.0 / M_PI;
			tmp4 << "name : " << this->model->GetJoints()[3]->GetName()  << " angel : " << this->model->GetJoints()[3]->Position() * 180.0 / M_PI;

			angleInfo.angleOne = tmp1.str();
			angleInfo.angleTwo = tmp2.str();
			angleInfo.angleThree = tmp3.str();
			angleInfo.angleFour = tmp4.str();

			this->publishAngleState.publish(angleInfo);
		}

		// a pointer that points to a model object
	private:
		physics::ModelPtr model;

		// 	// A joint controller object
		// 	// Takes PID value and apply angular velocity
		// 	//  or sets position of the angles
	private:
		physics::JointControllerPtr jointController;

	private:
		event::ConnectionPtr updateConnection;

		// // 	// PID object
	private:
		common::PID pid;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}