#include "lynxmotion_tm4c/tm4c_driver.h"
#include "math.h"
#include <algorithm>
#include <string>
#include "lynxmotion_tm4c/joint_maps.hpp"

namespace lynxmotion_tm4c
{

TM4CDriver::TM4CDriver( ros::NodeHandle &nh ) :
  nh( nh )
{
  for( int i = 0; i < 32; i++ )
    channels[i] = NULL;

  ros::NodeHandle priv_nh( "~" );

  priv_nh.param<std::string>( "port", port, "/dev/ttyACM0" );
  priv_nh.param<int>( "baud", baud, 115200 );
  priv_nh.param<bool>( "publish_joint_states", publish_joint_states, true );

  // 180 degree servos seem to have a slightly greater range than 180 degrees.
  // This parameter allows for scaling the range for attempting to get
  // it to be closer to 180 degrees, or 90 degrees for 90 degree servos.
  priv_nh.param<double>( "range_scale", range_scale, 1.0 );

  if ( range_scale > 1 || range_scale <= 0 )
    range_scale = 1.0;
  scale = range_scale * 2000.0 / M_PI;

  // Parse joints ros param
  XmlRpc::XmlRpcValue joints_list;
  if( priv_nh.getParam( "joints", joints_list ) )
  {
    ROS_ASSERT( joints_list.getType( ) == XmlRpc::XmlRpcValue::TypeStruct );

    XmlRpcValueAccess joints_struct_access( joints_list );
    XmlRpc::XmlRpcValue::ValueStruct joints_struct = joints_struct_access.getValueStruct( );

    XmlRpc::XmlRpcValue::ValueStruct::iterator joints_it;

    for( joints_it = joints_struct.begin( ); joints_it != joints_struct.end( ); joints_it++ )
    {
      Joint *joint = new Joint;
      joint->name = static_cast<std::string>( joints_it->first );

      std::string joint_graph_name = "joints/" + joint->name + "/";

      priv_nh.param<int>( joint_graph_name + "channel", joint->properties.channel, 0 );

      // Channel must be between 0 and 31, inclusive
      ROS_ASSERT( joint->properties.channel >= 0 );
      ROS_ASSERT( joint->properties.channel <= 31 );

      priv_nh.param<double>( joint_graph_name + "max_angle", joint->properties.max_angle, M_PI_2 );
      priv_nh.param<double>( joint_graph_name + "min_angle", joint->properties.min_angle, -M_PI_2 );
      priv_nh.param<double>( joint_graph_name + "offset_angle", joint->properties.offset_angle, joint->properties.offset_angle );
      priv_nh.param<double>( joint_graph_name + "default_angle", joint->properties.default_angle, joint->properties.offset_angle );
      priv_nh.param<bool>( joint_graph_name + "initialize", joint->properties.initialize, true );
      priv_nh.param<bool>( joint_graph_name + "invert", joint->properties.invert, false );

      // Make sure no two joints have the same channel
      ROS_ASSERT( channels[joint->properties.channel] == NULL );

      // Make sure no two joints have the same name
      ROS_ASSERT( joints_map.find( joint->name ) == joints_map.end( ) );

      channels[joint->properties.channel] = joint;
      joints_map[joint->name] = joint;
    }
  }
  else
  {
    ROS_FATAL( "No joints were given" );
    ROS_BREAK( );
  }

  // Parse controllers ros param
  XmlRpc::XmlRpcValue controllers_list;
  if( priv_nh.getParam( "controllers", controllers_list ) )
  {
    ROS_ASSERT( controllers_list.getType( ) == XmlRpc::XmlRpcValue::TypeStruct );

    // Get the controllers ValueStruct
    XmlRpcValueAccess controllers_struct_access( controllers_list );
    XmlRpc::XmlRpcValue::ValueStruct controllers_struct = controllers_struct_access.getValueStruct( );

    XmlRpc::XmlRpcValue::ValueStruct::iterator controllers_it;

    // For each controller, parse its type and the joints associated with the controller
    for( controllers_it = controllers_struct.begin( ); controllers_it != controllers_struct.end( ); controllers_it++ )
    {
      Controller *controller = new Controller;
      controller->name = static_cast<std::string>( controllers_it->first );

      std::string controller_graph_name = "controllers/" + controller->name + "/";

      std::string controller_type;
      priv_nh.param<std::string>( controller_graph_name + "type", controller_type, "joint_controller" );

      // Validate the controller type
      if( controller_type == "joint_controller" )
        controller->type = ControllerTypes::JointController;
      else if( controller_type == "diff_drive_controller" )
        controller->type = ControllerTypes::DiffDriveController;
      else
      {
        ROS_FATAL( "Unknown controller type [%s] for controller [%s]",
          controller_type.c_str( ), controller->name.c_str( ) );
        delete controller;
        ROS_BREAK( );
      }

      priv_nh.param<bool>( controller_graph_name + "publish_joint_states", controller->publish_joint_states, true );

      // Get publish rate
      priv_nh.param<double>( controller_graph_name + "publish_rate", controller->publish_rate, 10.0 );
      if( controller->publish_rate <= 0.0 )
      {
        controller->expected_publish_time = 0.0;
        controller->publish_joint_states = false;
      }
      else
        controller->expected_publish_time = ( 1.0 / controller->publish_rate );

      // Make sure the controller has joints
      if( priv_nh.getParam( controller_graph_name + "joints", joints_list ) )
      {
        ROS_ASSERT( joints_list.getType( ) == XmlRpc::XmlRpcValue::TypeArray );

        // Get joints array
        XmlRpcValueAccess joints_array_access( joints_list );
        XmlRpc::XmlRpcValue::ValueArray joints_array = joints_array_access.getValueArray( );

        // Parse the joint names and verify the joint exists
        for( unsigned int i = 0; i < joints_array.size( ); i++ )
        {
          std::string joint_name = static_cast<std::string>( joints_array[i] );

          ROS_INFO("slapping the joint on the joint stack: %s", joint_name.c_str());

          if( joints_map.find( joint_name ) != joints_map.end( ) )
          {
            controller->joints.push_back( joints_map[joint_name] );
          }
          else // joint doesn't exist
          {
            ROS_FATAL( "Joint [%s] for controller [%s] does not exist",
              joint_name.c_str( ), controller->name.c_str( ) );
            delete controller;
            ROS_BREAK( );
          }
        }
      }
      else // No joints were provided
      {
        ROS_FATAL( "Controller [%s] has no joints listed.", controller->name.c_str( ) );
        delete controller;
        ROS_BREAK( );
      }

      controllers.push_back( controller );
      controllers_map[controller->name] = controller;
    }
  }
  else
  {
    /*!
     * \todo Instead of throwing an error if no controllers are give,
     *       maybe put all joints into a 'global' controller that can
     *       be controlled over the topic cmd_joint_traj.
     */
    ROS_FATAL( "No controllers were given" );
    ROS_BREAK( );
  }

  relax_joints_service = nh.advertiseService( "relax_joints", &TM4CDriver::relaxJointsCallback, this );
}

TM4CDriver::~TM4CDriver( )
{
  /* stop( ); */

  for( int i = 0; i < 32; i++ )
    if( channels[i] )
      delete channels[i];

  while( !controllers.empty( ) )
  {
    Controller *controller = controllers.back( );
    controllers.pop_back( );
    delete controller;
  }
}

bool TM4CDriver::init( )
{
  TM4C::ServoCommand *cmd;
  bool success = true;

  // Initialize each controller
  for( unsigned int i = 0; i < controllers.size( ); i++ )
  {
    ROS_DEBUG( "Initializing controller %s", controllers[i]->name.c_str( ) );

    // Only initialize the controller if it's a joint controller
    if( controllers[i]->type == ControllerTypes::JointController )
    {
      cmd = new TM4C::ServoCommand[controllers[i]->joints.size( )];

      for( unsigned int j = 0; j < controllers[i]->joints.size( ); j++ )
      {
        Joint *joint = controllers[i]->joints[j];

        if( joint->properties.initialize )
        {
          cmd[j].ch = joint->properties.channel;
          cmd[j].pw = ( unsigned int )( scale * ( joint->properties.default_angle - joint->properties.offset_angle ) + 1500 + 0.5 );


          ROS_INFO( "Initializing channel %d to pulse width %d", cmd[j].ch, cmd[j].pw );

          if( joint->properties.invert )
            cmd[j].pw = 3000 - cmd[j].pw;

          if( cmd[j].pw < 500 )
            cmd[j].pw = 500;
          else if( cmd[j].pw > 2500 )
            cmd[j].pw = 2500;
        }
      }

      // Send command
      if( !tm4c_dev.move_servo( cmd, controllers[i]->joints.size( ) ) )
      {
        ROS_ERROR( "Failed initializing controller %s", controllers[i]->name.c_str( ) );
        success = false;
      }

      delete[] cmd;
    }
  }

  return success;
}

bool TM4CDriver::relaxJoints( )
{
  ROS_INFO( "Relaxing joints" );

  if( tm4c_dev.is_connected( ) )
  {
    for( unsigned int i = 0; i < 32; i++ )
    {
      if( channels[i] != NULL )
        tm4c_dev.discrete_output( i, TM4C::Low );
    }
  }

  return true;
}

bool TM4CDriver::relaxJointsCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
{
  return relaxJoints( );
}

bool TM4CDriver::spin( )
  {
    bool result = true;

    if( start( ) && init( ) ) {
    // if (tm4c_dev.open_port(port.c_str(), baud)) {
        // ROS_INFO( "HERSHAL: Spinning..." );

        ros::Rate loop_rate(10);
        while(ros::ok()) {
            update();
            // std::string message = "test\r\n";
            // tm4c_dev.send_message(message.c_str(), message.size());
            ros::spinOnce( );
            loop_rate.sleep();
          }
      } else {
        ROS_ERROR( "HERSHAL: Failed to start" );
        result = false;
      }

    /* stop( ); */

    return result;
}

bool TM4CDriver::start( )
{
  ROS_INFO( "Starting TM4CDriver" );

  // Start device
  if( !tm4c_dev.open_port( port.c_str( ), baud ) )
    return false;

  // std::string version = tm4c_dev.get_version( );
  // if( version.empty( ) )
  // {
  //  ROS_ERROR( "Unable to get software version" );
  //  ROS_INFO( "Verify the baud rate is set to the correct value" );
  //  return false;
  // }

  // ROS_INFO( "TM4C Software Version: %s", version.c_str( ) );

  // Subscribe and advertise for every controller
  for( unsigned int i = 0; i < controllers.size( ); i++ )
  {
    joint_state_pubs_map[controllers[i]->name] = nh.advertise<sensor_msgs::JointState>( controllers[i]->name + "/joint_states", 1 );
    joint_subs.push_back( nh.subscribe( controllers[i]->name + "/command", 1, &TM4CDriver::jointCallback, this ) );
  }

  return true;
}

void TM4CDriver::stop( )
{
  ROS_INFO( "Stopping TM4CDriver" );

  relaxJoints( );

  nh.shutdown( );

  joint_state_pubs_map.clear( );

  joint_subs.clear( );

  tm4c_dev.close_port( );
}

void TM4CDriver::update( )
{
  current_time = ros::Time::now( );

  if( publish_joint_states )
  {
    publishJointStates( );
  }

  last_time = current_time;
}


float lerp(float x, float x_min, float x_max, float y_min, float y_max) {
  if (x > x_max) { return y_max; }
  else if (x < x_min) { return y_min; }
  return y_min + (y_max - y_min)*((x - x_min)/(x_max - x_min));
}

void TM4CDriver::publishJointStates( )
{
  for( unsigned int i = 0; i < controllers.size( ); i++ )
  {
    if( controllers[i]->publish_joint_states &&
      fabs( ( controllers[i]->last_publish_time - current_time ).toSec( ) ) >= controllers[i]->expected_publish_time)
    {
      sensor_msgs::JointState joints;
      joints.header.stamp = current_time;

      for( unsigned int j = 0; j < controllers[i]->joints.size( ); j++ )
      {
        joints.name.push_back( controllers[i]->joints[j]->name );

        int pw = tm4c_dev.query_pulse_width( controllers[i]->joints[j]->properties.channel );

        if( controllers[i]->joints[j]->properties.invert )
          pw = 3000 - pw;

        // ROS_INFO( "Pulse width for joint [%s] is %d", controllers[i]->joints[j]->name.c_str( ), pw );
        //double angle = M_PI_2 * ( ( double )pw - controllers[i]->joints[j]->properties.default_angle ) / 1000.0;
        double angle = ( ( double ) pw - 1500.0 ) / scale + controllers[i]->joints[j]->properties.offset_angle;

        // ROS_INFO( "Angle calculated for joint [%s] is %f", controllers[i]->joints[j]->name.c_str( ), angle );

        //if( controllers[i]->joints[j]->properties.invert )
        //  angle *= -1;

        joints.position.push_back( angle );
      }

      joint_state_pubs_map[controllers[i]->name].publish( joints );

      controllers[i]->last_publish_time = current_time;
    }
  }
}

void TM4CDriver::jointCallback( const ros::MessageEvent<trajectory_msgs::JointTrajectory const>& event )
{
  ros::M_string connection_header = event.getConnectionHeader( );
  const trajectory_msgs::JointTrajectoryConstPtr &msg = event.getMessage( );

  std::string topic = connection_header["topic"];

  if( topic.empty() ) {
    ROS_ERROR( "The connection header topic is empty" );
    return;
  }

  // Remove beginning '/'
  if( topic[0] == '/')
    topic.erase( 0, 1 );

  // Extract the controller name from the topic
  std::string::iterator it = find( topic.begin( ), topic.end( ), '/' );
  if( it != topic.end( ) )
    topic.erase( it, topic.end( ) );

  // Validate the controller name
  if( controllers_map.find( topic ) == controllers_map.end() ) {
    ROS_ERROR( "[%s] is not a valid controller name.", topic.c_str( ) );
    return;
  }

  int num_joints = controllers_map[topic]->joints.size( );
  TM4C::ServoCommand *cmd = new TM4C::ServoCommand[num_joints];

  bool invalid = false;

  double stop_duration = 0;
  if (msg->points[0].time_from_start.toSec() > 0)
    stop_duration = msg->points[0].time_from_start.toSec();
  double first_time = ros::Time::now().toSec();

  for(unsigned int i = 0; i < msg->joint_names.size() && !invalid; i++) {
    if(joints_map.find(msg->joint_names[i]) != joints_map.end()) {

      Joint *joint = joints_map[msg->joint_names[i]];
      double angle = msg->points[0].positions[i];

      // Validate the commanded position (angle)
      if( angle >= joint->properties.min_angle && angle <= joint->properties.max_angle ) {
        cmd[i].ch = joint->properties.channel;

        if (i == 1) {
          // cmd[i].pw = (unsigned int)(angle*joint->properties.slope + joint->properties.intercept);
          cmd[i].pw = (int)interpolator.get(msg->points[0].positions[1] * (180/M_PI),
                                            msg->points[0].positions[2] * (180/M_PI),
                                            m_shl);
        } else if (i == 2) {
          cmd[i].pw = (int)interpolator.get(msg->points[0].positions[1] * (180/M_PI),
                                            msg->points[0].positions[2] * (180/M_PI),
                                            m_elb);
        } else {
          cmd[i].pw = (unsigned int)
            (lerp((float)(angle), 0.0 + joint->properties.offset_angle, 3.14, 600.0, 2400.0));
        }
        ROS_INFO("setting joint %i angle to %i", i, cmd[i].pw);
        /* cmd[i].pw = ( unsigned int )( scale * ( angle - joint->properties.offset_angle ) + 1500 + 0.5 ); */
        if( joint->properties.invert )
          cmd[i].pw = 3000 - cmd[i].pw;
        /* if( cmd[i].pw < 500 ) */
        /*  cmd[i].pw = 500; */
        /* else if( cmd[i].pw > 2500 ) */
        /*  cmd[i].pw = 2500; */

        if( msg->points[0].velocities.size( ) >= i && msg->points[0].velocities[i] > 0 )
          cmd[i].spd = scale * msg->points[0].velocities[i];
      } else {
        invalid = true;
        ROS_ERROR( "The given position [%f] for joint [%s] is invalid 2", angle, joint->name.c_str( ) );
      }
    } else {
      invalid = true;
      ROS_ERROR( "Joint [%s] does not exist", msg->joint_names[i].c_str( ) );
    }
  }

  if(!invalid)
  {
    // Send command
    if( !tm4c_dev.move_servo( cmd, num_joints ) )
      ROS_ERROR( "Failed sending joint commands to controller" );
  }

  delete[] cmd;
}
}
