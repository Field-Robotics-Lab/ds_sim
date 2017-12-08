/* A class to convert gazebo depth to ROS depth
 *
 * Also, may add noise (up to you)
 * 
 * 2017 Nov 22
 * Ian Vaughn
 */

#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <ds_sensor_msgs/InsData.h>
#include <string>
#include <geometry_msgs/QuaternionStamped.h>

#include "../gazebo_src/dsros_ins.hh"

namespace gazebo {

class dsrosRosInsSensor : public SensorPlugin
{
public:

  /// \brief Constructor
  dsrosRosInsSensor();

  /// \brief Destructor
  virtual ~dsrosRosInsSensor();

  /// \brief Load the sensor.
  /// \param sensor_ pointer to the sensor.
  /// \param sdf_ pointer to the sdf config file.
  virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

protected:
  /// \brief Update the sensor
  virtual void UpdateChild(const gazebo::common::UpdateInfo &_info);

private:

  double GaussianKernel(double mu, double sigma);
  double LoadNoise(const std::string& tag, double unit) const;
  bool LoadParameters();

  /// \brief ROS Node Handle
  ros::NodeHandle* node;

  /// \brief INS data publisher
  ros::Publisher ins_publisher;

  /// \brief Attitude data publisher
  ros::Publisher att_publisher;

  ds_sensor_msgs::InsData ins_msg;
  geometry_msgs::QuaternionStamped att_msg;

  /// \brief last time on which the data was published.
  common::Time last_time;

  /// \brief Pointer to the update event connection.
  gazebo::event::ConnectionPtr connection;

  /// \brief Pointer to the sensor.
  std::shared_ptr<sensors::DsrosInsSensor> sensor;

  /// \brief Pointer to the sdf config file.
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string ins_topic_name;
  std::string att_topic_name;
  std::string frame_name;

  unsigned int seed;
  double update_rate;
  double noisePR, noiseY, noiseVel, noiseAngVel, noiseAcc, noiseLat;

  // actual core data
  common::Time data_time;
  std::string entity_name;
  ignition::math::Quaterniond world2ll;
  ignition::math::Quaterniond orientation;
  ignition::math::Vector3d angular_velocity;
  ignition::math::Vector3d linear_velocity;
  ignition::math::Vector3d linear_accel;
  double pitch, roll, heading;
  double latitude;
};

}; // namespace gazebo
