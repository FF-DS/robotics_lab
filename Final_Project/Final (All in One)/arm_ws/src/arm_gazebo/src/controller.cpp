#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
#include "arm_gazebo/JointAngleState.h"
#include "arm_gazebo/JointAngles.h"
#include "arm_gazebo/GripperState.h"
#include "arm_gazebo/IK.h"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{	
	public:

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			// initialize ros if not initalized so far //
			if( !ros::isInitialized() ){
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "JointAnglePublisherSubscriber");
			}

			// Store the pointer to the model
			this->model = _parent;


			// prepare robot
			this->prepareRobot();


			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));

			
			// ----------------- ros part -------------------- //

			// ------------ initalize arm joints publisher ------------- //
			ros::NodeHandle arm_p, arm_s, gripper_s;
			this->publishAngleState = arm_p.advertise<arm_gazebo::JointAngleState>("JointAngles", 1000);
			ros::Rate loop_rate(10);

			// ------------ initalize arm joints subscriber ------------- // 
			this->subscribeAngleState = arm_s.subscribe("UpdateJointAngles", 1000, &ModelPush::updateJointAngleCallback, this);
			// ------------  end ------------- // 

			// // ------------ initalize gripper state subscriber ------------- // 
			this->subscribeGripperState = gripper_s.subscribe("GripperState", 1000, &ModelPush::gripperControllerCallback, this);
			// // ------------  end ------------- // 
			
		}

	public:
		void prepareRobot(){
			common::PID pid = common::PID(400, 50, 250);

			std::string name = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			name = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			name = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			name = this->model->GetJoint("arm4_arm5_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			name = this->model->GetJoint("arm5_gripper1_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			name = this->model->GetJoint("gripper1_gripper2_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			name = this->model->GetJoint("gripper1_gripper3_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionPID(name, pid);
			this->model->GetJointController()->SetPositionTarget(name, 0);

			this->releaseObject();
		}

		// Called by the world update start event
	public:
		void OnUpdate()
		{
			// ---- Publish to JointAngels topics ---- //
			arm_gazebo::JointAngleState angleInfo;
			std::stringstream tmp1, tmp2, tmp3, tmp4;
			
			tmp1 << "name : " << this->model->GetJoints()[1]->GetName()  << " angel : " << this->model->GetJoints()[1]->Position(0) * 180.0 / M_PI;
			tmp2 << "name : " << this->model->GetJoints()[2]->GetName()  << " angel : " << this->model->GetJoints()[2]->Position(0) * 180.0 / M_PI;
			tmp3 << "name : " << this->model->GetJoints()[3]->GetName()  << " angel : " << this->model->GetJoints()[3]->Position(0) * 180.0 / M_PI;
			tmp4 << "name : " << this->model->GetJoints()[4]->GetName()  << " angel : " << this->model->GetJoints()[4]->Position(0) * 180.0 / M_PI;

			angleInfo.angleOne = tmp1.str();
			angleInfo.angleTwo = tmp2.str();
			angleInfo.angleThree = tmp3.str();
			angleInfo.angleFour = tmp4.str();

			this->publishAngleState.publish(angleInfo);
			// --- end of publisher -- //
		}


		// --- arm joint subscriber  callback --- //
	public:
		void updateJointAngleCallback(const arm_gazebo::JointAngles::ConstPtr& angleState)
		{
			this->model->GetJoints()[1]->SetPosition(0, angleState->angleOne * M_PI/ 180.0 );
			this->model->GetJoints()[2]->SetPosition(0, angleState->angleTwo * M_PI/ 180.0 );
			this->model->GetJoints()[3]->SetPosition(0, angleState->angleThree * M_PI/ 180.0 );
			this->model->GetJoints()[4]->SetPosition(0, angleState->angleFour * M_PI/ 180.0 );
		}


		// --- gripper controller subscriber  callback --- //
	public:
		void gripperControllerCallback(const arm_gazebo::GripperState::ConstPtr& gripperState)
		{
			std::vector<float> links = { 0.1, 0.05, 2, 1, .5, .2, 0.05 }; 

			this->updateArmJointState( links, gripperState->ee);

			if( gripperState->catchCommand ){
				this->catchObject();
			}else{
				this->releaseObject();
			}
		}

	public:
		void catchObject()
		{
			std::string name = this->model->GetJoint("gripper1_gripper2_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionTarget(name, 45 * M_PI/ 180.0 );

			name = this->model->GetJoint("gripper1_gripper3_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionTarget(name, -45 * M_PI/ 180.0);
		}

	public:
		void releaseObject()
		{
			std::string name = this->model->GetJoint("gripper1_gripper2_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionTarget(name, -135 * M_PI/ 180.0);

			name = this->model->GetJoint("gripper1_gripper3_joint")->GetScopedName();
			this->model->GetJointController()->SetPositionTarget(name, 135 * M_PI/ 180.0);
		}

	public:
		void updateArmJointState(std::vector<float> links, std::vector<float> ee)
		{
			ros::NodeHandle n;
			ros::ServiceClient ikService = n.serviceClient<arm_gazebo::IK>("IkService");

			arm_gazebo::IK srv;
			srv.request.links = links;
			srv.request.ee = ee;
			
			if (ikService.call(srv)){
				std::string name = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
				this->model->GetJointController()->SetPositionTarget(name, srv.response.angles[0]);

				name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
				this->model->GetJointController()->SetPositionTarget(name, srv.response.angles[1]);

				name = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
				this->model->GetJointController()->SetPositionTarget(name, srv.response.angles[2]);

				name = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();
				this->model->GetJointController()->SetPositionTarget(name, srv.response.angles[3]);

				name = this->model->GetJoint("arm4_arm5_joint")->GetScopedName();
				this->model->GetJointController()->SetPositionTarget(name, srv.response.angles[4]);

				name = this->model->GetJoint("arm5_gripper1_joint")->GetScopedName();
				this->model->GetJointController()->SetPositionTarget(name, srv.response.angles[5]);
			}
		}


		// a pointer that points to a model object
	private:
		physics::ModelPtr model;

	private:
		event::ConnectionPtr updateConnection;


		// ros part 
	private: 
		ros::Publisher publishAngleState;
	
	private: 
		ros::Subscriber subscribeAngleState;

	private: 
		ros::Subscriber subscribeGripperState;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}