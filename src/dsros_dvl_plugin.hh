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
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <ds_sensor_msgs/DvlData.h>
#include <string>

#include "../gazebo_src/dsros_dvl.hh"

namespace gazebo {

class dsrosRosDvlSensor : public SensorPlugin
{
public:

  /// \brief Constructor
  dsrosRosDvlSensor();

  /// \brief Destructor
  virtual ~dsrosRosDvlSensor();

  /// \brief Load the sensor.
  /// \param sensor_ pointer to the sensor.
  /// \param sdf_ pointer to the sdf config file.
  virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

protected:
  /// \brief Update the sensor
  virtual void UpdateChild(const gazebo::common::UpdateInfo &_info);

private:

  double GaussianKernel(double mu, double sigma);
  bool LoadParameters();

  /// \brief ROS Node Handle
  ros::NodeHandle* node;

  /// \brief Depth publisher
  ros::Publisher dvl_data_publisher;

  ds_sensor_msgs::DvlData msg;

  // temporary stuff to make sure we're point our beams correctly
  ros::Publisher pt_data_publisher;
  std::string pointcloud_frame;
  sensor_msgs::PointCloud pt_msg;

  /// \brief last time on which the data was published.
  common::Time last_time;

  /// \brief Pointer to the update event connection.
  gazebo::event::ConnectionPtr connection;

  /// \brief Pointer to the sensor.
  std::shared_ptr<sensors::DsrosDvlSensor> sensor;

  /// \brief Pointer to the sdf config file.
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string frame_name;

  unsigned int seed;
  double update_rate;
  double gaussian_noise_vel;
  double gaussian_noise_range;
};

}; // namespace gazebo
