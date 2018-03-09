#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include "tf/transform_listener.h"
#include <tams_ft_calibration_msgs/Calibration.h>
#include <tams_ft_calibration_msgs/SetToolWeight.h>
#include <tams_ft_calibration_msgs/Payload.h>

class Calibration
{
public:
  Calibration()
  {
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string wrench_topic;
    std::string cart_controller_topic;
    pn.param("wrench_topic", wrench_topic, std::string("robotiq_force_torque_wrench"));
    pn.param("offsets", offsets_, {0,0,0,0,0,0});
    pn.param("gains", gains_, {1,1,1,1,1,1});
    pn.param("verbose", verbose_, false);

    std::vector<double> payload_mass;
    std::vector<double> payload_offset_x;
    std::vector<double> payload_offset_y;
    std::vector<double> payload_offset_z;
    pn.param("payload_mass", payload_mass, {0.0});
    pn.param("payload_mass", payload_offset_x, {0.0});
    pn.param("payload_mass", payload_offset_y, {0.0});
    pn.param("payload_mass", payload_offset_z, {0.0});

    // all vectors need to be the same size
    if (payload_mass.size() != payload_offset_x.size() ||
        payload_offset_x.size() != payload_offset_y.size() ||
        payload_offset_y.size() != payload_offset_z.size())
    {
      ROS_ERROR("The number of provided payload parameters differ.");
      ros::shutdown();
    }

    tams_ft_calibration_msgs::Payload onePayload;
    for (int i = 0; i < payload_mass.size(); i++)
    {
      onePayload.mass     = payload_mass[i];
      onePayload.offset.x = payload_offset_x[i];
      onePayload.offset.y = payload_offset_y[i];
      onePayload.offset.z = payload_offset_z[i];

      tool_weights_.push_back( onePayload );
    }

    if(offsets_.size() != 6 || gains_.size() != 6)
    {
      ROS_ERROR("Wrong number of gains and offsets as parameter. 6 offsets and 6 gains are required. Using 1.0 for all gains and 0.0 for all offsets.");

      gains_.resize(6);
      offsets_.resize(6);

      std::fill(gains_.begin(), gains_.end(), 1.0);
      std::fill(offsets_.begin(), offsets_.end(), 0.0);
    }

    wrench_sub_ = n.subscribe(wrench_topic, 1, &Calibration::wrench_callback, this);
    calibrated_internal_wrench_pub_ = n.advertise<geometry_msgs::WrenchStamped>("calibrated_wrench/internal", 1);
    calibrated_external_wrench_pub_ = n.advertise<geometry_msgs::WrenchStamped>("calibrated_wrench/external", 1);
    calibrated_wrench_pub_ = n.advertise<geometry_msgs::WrenchStamped>("calibrated_wrench/full", 1);

    set_calibration_server_ = n.advertiseService("set_calibration", &Calibration::set_calibration, this);
    set_tool_weight_server_ = n.advertiseService( "set_tool_weights", &Calibration::set_tool_weight, this );
  }

  void wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    geometry_msgs::WrenchStamped internal_wrench_msg;
    geometry_msgs::WrenchStamped external_wrench_msg;
    geometry_msgs::WrenchStamped wrench_msg;
    internal_wrench_msg.header = msg->header;
    external_wrench_msg.header = msg->header;
    wrench_msg.header = msg->header;

    // Calculating values for internal wrench
    internal_wrench_msg.wrench.force.x =  gains_[0] * (msg->wrench.force.x - offsets_[0]);
    internal_wrench_msg.wrench.force.y =  gains_[1] * (msg->wrench.force.y - offsets_[1]);
    internal_wrench_msg.wrench.force.z =  gains_[2] * (msg->wrench.force.z - offsets_[2]);
    internal_wrench_msg.wrench.torque.x =  gains_[3] * (msg->wrench.torque.x - offsets_[3]);
    internal_wrench_msg.wrench.torque.y =  gains_[4] * (msg->wrench.torque.y - offsets_[4]);
    internal_wrench_msg.wrench.torque.z =  gains_[5] * (msg->wrench.torque.z - offsets_[5]);
    calibrated_internal_wrench_pub_.publish(internal_wrench_msg);

    // Calculating values for external wrench
    estimate_tool_wrench(external_wrench_msg);
    calibrated_external_wrench_pub_.publish(external_wrench_msg);

    // Adding external to internal wrench
    wrench_msg.wrench.force.x = internal_wrench_msg.wrench.force.x + external_wrench_msg.wrench.force.x;
    wrench_msg.wrench.force.y = internal_wrench_msg.wrench.force.y + external_wrench_msg.wrench.force.y;
    wrench_msg.wrench.force.z = internal_wrench_msg.wrench.force.z + external_wrench_msg.wrench.force.z;
    wrench_msg.wrench.torque.x = internal_wrench_msg.wrench.torque.x + external_wrench_msg.wrench.torque.x;
    wrench_msg.wrench.torque.y = internal_wrench_msg.wrench.torque.y + external_wrench_msg.wrench.torque.y;
    wrench_msg.wrench.torque.z = internal_wrench_msg.wrench.torque.z + external_wrench_msg.wrench.torque.z;
    calibrated_wrench_pub_.publish(wrench_msg);
  }

  bool set_calibration(tams_ft_calibration_msgs::Calibration::Request& req,
                       tams_ft_calibration_msgs::Calibration::Response& res)
  {
    offsets_[0] = req.force_offsets.x;
    offsets_[1] = req.force_offsets.y;
    offsets_[2] = req.force_offsets.z;
    offsets_[3] = req.torque_offsets.x;
    offsets_[4] = req.torque_offsets.y;
    offsets_[5] = req.torque_offsets.z;

    gains_[0] = req.force_gains.x;
    gains_[1] = req.force_gains.y;
    gains_[2] = req.force_gains.z;
    gains_[3] = req.torque_gains.x;
    gains_[4] = req.torque_gains.y;
    gains_[5] = req.torque_gains.z;
    res.success = true;
    return true;
  }

  bool set_tool_weight(tams_ft_calibration_msgs::SetToolWeight::Request& req,
                       tams_ft_calibration_msgs::SetToolWeight::Response& res)
  {
    tool_weights_ = req.payloads;
    res.success = true;
    return true;
  }

  void estimate_tool_wrench( geometry_msgs::WrenchStamped& tool_wrench ) 
  {
    tf::StampedTransform world_sensor_transform;
    try {
  
      tf_.waitForTransform( "world", tool_wrench.header.frame_id, ros::Time(0), ros::Duration(0.2) );
      tf_.lookupTransform("world", tool_wrench.header.frame_id, ros::Time(0), world_sensor_transform);
      if (verbose_)  ROS_INFO( "got the world->sensor transform ok." );
    }
    catch( tf::TransformException& exception ) {
      ROS_ERROR_THROTTLE( 10, "estimateExternalWrench TransformListener failed: %s", exception.what() );
      tool_wrench.wrench.force.x = NAN;
      tool_wrench.wrench.force.y = NAN;
      tool_wrench.wrench.force.z = NAN;
      tool_wrench.wrench.torque.x = NAN;
      tool_wrench.wrench.torque.y = NAN;
      tool_wrench.wrench.torque.z = NAN;
      return;
    }
  
    // calculate vector projection of sensor-frame to gravity;
    // this assumes that gravity is (0,0,-1) in the world frame.
    //
    tf::Vector3 force( 0, 0, 0 );
    tf::Vector3 torque( 0, 0, 0 );
  
    for( int i=0; i < tool_weights_.size(); i++ ) {
      tams_ft_calibration_msgs::Payload load = tool_weights_[i];
      tf::Vector3 force_z  = tf::Vector3( 0, 0, 9.81*tool_weights_[i].mass );  // actually points upwards!
  
      tf::Vector3 offset_i = tf::Vector3( load.offset.x, load.offset.y, load.offset.z );
      tf::Vector3 force_i  = world_sensor_transform.getBasis().inverse() * force_z;
      tf::Vector3 torque_i = offset_i.cross( force_i );
  
      if (verbose_) {
        ROS_ERROR( "toolWeight %d  F: %7.4lf %7.4lf %7.4lf  M: %7.4lf %7.4lf %7.4lf",
                  i, force_i.getX(), force_i.getY(), force_i.getZ(), torque_i.getX(), torque_i.getY(), torque_i.getZ() );
      }
  
      force += force_i;
      torque += torque_i;
    }
  
    tool_wrench.wrench.force.x = force.getX();
    tool_wrench.wrench.force.y = force.getY();
    tool_wrench.wrench.force.z = force.getZ();
    tool_wrench.wrench.torque.x = torque.getX();
    tool_wrench.wrench.torque.y = torque.getY();
    tool_wrench.wrench.torque.z = torque.getZ();
  }

private:
  ros::Subscriber wrench_sub_;
  ros::Publisher calibrated_internal_wrench_pub_;
  ros::Publisher calibrated_external_wrench_pub_;
  ros::Publisher calibrated_wrench_pub_;
  std::vector<double> offsets_;
  std::vector<double> gains_;
  std::vector<tams_ft_calibration_msgs::Payload> tool_weights_;
  ros::ServiceServer set_calibration_server_;
  ros::ServiceServer set_tool_weight_server_;

  tf::TransformListener tf_;

  bool verbose_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_node");

  Calibration calib;
  ros::spin();
}
