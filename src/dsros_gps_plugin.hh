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
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

#include <gazebo/sensors/GpsSensor.hh>

namespace gazebo {

class dsrosRosGpsSensor : public SensorPlugin
{
public:

  /// \brief Constructor
  dsrosRosGpsSensor();

  /// \brief Destructor
  virtual ~dsrosRosGpsSensor();

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
  ros::Publisher publisher;

  sensor_msgs::NavSatFix gps_msg;

  /// \brief last time on which the data was published.
  common::Time last_time;

  /// \brief Pointer to the update event connection.
  gazebo::event::ConnectionPtr connection;

  /// \brief Pointer to the sensor.
  std::shared_ptr<gazebo::sensors::GpsSensor> sensor;

  /// \brief Pointer to the sdf config file.
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string frame_name;

  unsigned int seed;
  double update_rate;
    // noise, in meters, for
  double noiseLL_m, noiseZ_m;
    double min_altitude;

  // actual core data
  common::Time data_time;
  std::string entity_name;
    double latitude;
    double longitude;
    double altitude;
};

}; // namespace gazebo
