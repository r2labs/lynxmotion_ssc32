#ifndef LYNXMOTION_TM4C_TM4C_NODE_H
#define LYNXMOTION_TM4C_TM4C_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <XmlRpcValue.h>
#include <string>
#include <vector>
#include "tm4c.h"

namespace lynxmotion_tm4c
{

struct Joint
{
	struct JointProperties
	{
		int channel;
		double min_angle;
		double max_angle;
		double offset_angle; // this angle is considered to be 1500 uS
		double default_angle; // angle that the joint is initialized to (defaults to the offset_angle)
		bool initialize; // Indicates whether to initialize the servo to the default angle on startup.
		bool invert;
	};

	std::string name;
	JointProperties properties;
};

namespace ControllerTypes
{
	enum ControllerType
	{
		JointController,
		DiffDriveController
	};
}
typedef ControllerTypes::ControllerType ControllerType;

class TM4CDriver;

struct Controller
{
	std::string name;
	ControllerType type;
	std::vector<Joint*> joints; // Pointer to the joints in this controller
	bool publish_joint_states;
	double publish_rate;

	private:
		double expected_publish_time;
		ros::Time last_publish_time;
		friend class TM4CDriver;
};

class TM4CDriver
{
	public:
		TM4CDriver( ros::NodeHandle &nh );
		~TM4CDriver( );
		bool init( );
		bool relaxJoints( );
		bool relaxJointsCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response );
		bool spin( );
		bool start( );
		void stop( );
		void update( );

	private:
		void publishJointStates( );
		void jointCallback( const ros::MessageEvent<trajectory_msgs::JointTrajectory const>& event );

		ros::NodeHandle nh;
		ros::ServiceServer relax_joints_service;
		std::vector<ros::Subscriber> joint_subs;
		std::map<std::string, ros::Publisher> joint_state_pubs_map;

		Joint *channels[32];

		std::string port;
		int baud;
		bool publish_joint_states;
		double range_scale;
		double scale;
		std::vector<Controller*> controllers;
		std::map<std::string, Controller*> controllers_map;
		std::map<std::string, Joint*> joints_map;

		TM4C tm4c_dev;

		ros::Time current_time;
		ros::Time last_time;

		/*!
		 * \brief Class that gives access to an XmlRpcValue's ValueStruct or ValueArray.
		 */
		class XmlRpcValueAccess : private XmlRpc::XmlRpcValue
		{
			public:
				XmlRpcValueAccess( XmlRpc::XmlRpcValue xml_rpc_value ) :
					XmlRpc::XmlRpcValue( xml_rpc_value ) { }

				XmlRpc::XmlRpcValue::ValueStruct getValueStruct( )
				{
					assertStruct( );
					return *_value.asStruct;
				}

				XmlRpc::XmlRpcValue::ValueArray getValueArray( )
				{
					assertArray( size( ) );
					return *_value.asArray;
				}
		};
};

};

#endif // TM4C_TM4C_NODE_H
