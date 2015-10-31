#include <ros/ros.h>
#include "lynxmotion_tm4c/tm4c_driver.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "tm4c_node" );
	ros::NodeHandle nh;

	lynxmotion_tm4c::TM4CDriver tm4c( nh );

	tm4c.spin( );

	return 0;
}
