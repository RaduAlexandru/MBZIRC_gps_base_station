#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include <wgs_conversion.h>
#include <boost/thread.hpp>
#include <dji_sdk/GlobalPosition.h>

float snan= std::numeric_limits<float>::signaling_NaN();
typedef std::chrono::high_resolution_clock Clock;


class BaseStation  {

	public:
    BaseStation();
    virtual ~BaseStation(void);
	protected:
	  void publish_reference( const ros::TimerEvent& evt );
		void gps_callback( const dji_sdk::GlobalPosition::ConstPtr& position_msg );
	private:
		// ros::NodeHandle n;
		// boost::thread com_thread;
	  // tf::Transform m_field_center;
    std::string m_world_frame_id;
    std::string m_gps_frame_id;
    std::string m_odom_frame_id;
    std::string m_footprint_frame_id;
    std::string m_base_frame_id;

		tf::StampedTransform m_last_valid_field;
    tf::Transform m_field_in_groundstation_frame;
	  ros::Publisher m_pose_publisher;
	  ros::Timer m_timer;
		ros::Subscriber gps_sub;
    double yaw_deg;

		// void com_data();
    void initialize_position();

};
