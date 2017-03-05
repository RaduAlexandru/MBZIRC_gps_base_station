#include "base_station.h"


BaseStation::BaseStation(void)
{
  ros::NodeHandle n( "~" );

  m_gps_frame_id = "earth";
  m_world_frame_id = "world";
  m_odom_frame_id = "odom";
  m_footprint_frame_id = "base_footprint";
  m_base_frame_id = "base_link";


  //Set the position of the field center relative to the groundstation
  initialize_position();
  //  m_field_in_groundstation_frame.setIdentity();
  //m_field_in_groundstation_frame.setOrigin(tf::Point( 10, 0, 0. ));
  //yaw=0;



  //subscribe to the /dji_sdk/global_position //TODO get the topic name from the parameters inf the elaunch file
  gps_sub = n.subscribe("/dji_sdk/global_position", 1000, &BaseStation::gps_callback, this);




  // m_field_center.setRotation( tf::createQuaternionFromYaw( yaw_deg * M_PI / 180. ) );

  // comm. infrastructure
  m_pose_publisher = n.advertise< nav_msgs::Odometry >( "/base_station/reference_pose", 1 );
  m_timer = n.createTimer( ros::Duration( 0.1 ), &BaseStation::publish_reference, this );

  // while(ros::ok()){
  //   ros::spinOnce();
  // }

  // ros::spin();

}

BaseStation::~BaseStation(void){
}

//Initialize the rigid transform between ground station and field center
void BaseStation::initialize_position(){
   ros::NodeHandle ph( "~" );

   m_field_in_groundstation_frame.setIdentity();

  // position in either m or latitude, longitude, height
  bool position_initialized = false;
  if( ph.hasParam( "x" ) && ph.hasParam( "y" ) && ph.hasParam( "z" ) ) {
    double x, y, z;
    if( ph.getParam( "x", x ) && ph.getParam( "y", y ) && ph.getParam( "z", z ) ) {
      position_initialized = true;
      std::cout << "rigid transform is " << x << " " << y << " " << z << std::endl;
      m_field_in_groundstation_frame.setOrigin( tf::Vector3( x, y, z ) );
    }
  }
  else if( ph.hasParam( "lat" ) && ph.hasParam( "lon" ) && ph.hasParam( "height" ) ) {
    double lat, lon, z;
    if( ph.getParam( "lat", lat ) && ph.getParam( "lon", lon ) && ph.getParam( "height", z ) ) {
      position_initialized = true;
      double x, y;
      mod_utils::LL2UTM( lat, lon, x, y );
      m_field_in_groundstation_frame.setOrigin( tf::Vector3( x, y, z ) );
    }
  }

  if( !position_initialized ) {
    ROS_FATAL( "Parameter x, y, z OR lat, lon, height missing." );
    exit( -1 );
  }

  if( !ph.getParam( "yaw_deg", yaw_deg ) ) {
    ROS_FATAL( "Parameter yaw_deg missing." );
    exit( -1 );
  }


}

void BaseStation::publish_reference( const ros::TimerEvent& /*evt*/ ) {
  // geometry_msgs::PoseStamped field_center_msg;
  nav_msgs::Odometry field_center_msg;
  //field_center_msg.header.stamp = ros::Time::now();
  field_center_msg.header.stamp=m_last_valid_field.stamp_;
  field_center_msg.header.frame_id = m_gps_frame_id;  //TODO the field center needs to be the world
  field_center_msg.child_frame_id = m_base_frame_id;
  //tf::poseTFToMsg( m_last_valid_field, field_center_msg.pose  );
  tf::poseTFToMsg( m_last_valid_field, field_center_msg.pose.pose   );
  m_pose_publisher.publish( field_center_msg );
  //std::cout << "publishing" << '\n';

}


// void BaseStation::com_data (){
//   ros::NodeHandle n( "~" );
//   gps_sub = n.subscribe("/mbzirc_msg_bridge/gps_localization", 1000, &BaseStation::gps_callback, this);
//   ros::spin();
// }

void BaseStation::gps_callback(const dji_sdk::GlobalPosition::ConstPtr& position_msg) {
  //std::cout << "got gps info" << '\n';
  //std::cout << "health is " << (int)position_msg->health << std::endl;


  int health= (int)position_msg->health;

  // ====== GLOBAL POSITION FROM MSG =====
  double x, y, z;
  mod_utils::LL2UTM( position_msg->latitude, position_msg->longitude, x, y );
  z = position_msg->altitude;


  tf::Transform base_utm;
  base_utm.setOrigin(tf::Vector3( x, y, z ));
  //tf::StampedTransform base_utm = tf::StampedTransform(tf, position_msg->header.stamp, m_gps_frame_id, m_base_frame_id );



  //get the field center in UTM coordinates
  tf::StampedTransform field_frame = tf::StampedTransform( m_field_in_groundstation_frame*base_utm, position_msg->header.stamp, m_gps_frame_id, m_base_frame_id) ;
  //tf::Quaternion q = tf::createQuaternionFromRPY( 0., 0., yaw );
  field_frame.setRotation(tf::createQuaternionFromYaw( yaw_deg * M_PI / 180. ) );

  //field_frame.stamp_=position_msg->header.stamp;


  //if it's stable then update it
  if (health >=3){
    m_last_valid_field=field_frame;
  }else{
    std::cout << "health is too low" << std::endl;
  }



  /*
  if (health_is_good){
    multiply position_msg with transform_fg;
    set rotation to yaw;
    m_last_valid_field=this transform we just calcualted
  }
  */

}


int main(int argc,char* argv[]){

  ros::init( argc, argv, "base_station" );

  BaseStation bs;




  ros::spin();



  // ros::init( argc, argv, "base_station" );
  // ros::NodeHandle n;
  // ros::Rate loopRate( 30 );
  //
  // BaseStation bs;
  //
  // while( n.ok() ) {
  //   ros::spinOnce();
  //   bs.publish_reference();
  //   loopRate.sleep();
  //
  // }


  return 0;

}
