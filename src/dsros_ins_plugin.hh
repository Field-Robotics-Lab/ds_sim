/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
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
#include <ds_sensor_msgs/Ins.h>
#include <ds_sensor_msgs/Gyro.h>
#include <ds_sensor_msgs/PhinsStdbin3.h>
#include <ros/ros.h>
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

  /// \brief Publish gyro messages based on INS data
  ros::Publisher gyro_publisher;

  /// \brief Attitude data publisher
  ros::Publisher att_publisher;

  /// \brief Phins binary publisher
  ros::Publisher phinsbin_publisher;

  ds_sensor_msgs::Ins ins_msg;
  ds_sensor_msgs::Gyro gyro_msg;
  ds_sensor_msgs::PhinsStdbin3 phinsbin_msg;
  geometry_msgs::QuaternionStamped att_msg;

  /// \brief last time on which the data was published.
  common::Time last_time;

  /// \brief Time the last phins binary data was published.
  common::Time last_phinsbin_time;

  /// \brief Pointer to the update event connection.
  gazebo::event::ConnectionPtr connection;

  /// \brief Pointer to the sensor.
  std::shared_ptr<sensors::DsrosInsSensor> sensor;

  /// \brief Pointer to the sdf config file.
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string ins_topic_name;
  std::string gyro_topic_name;
  std::string att_topic_name;
  std::string phinsbin_topic_name;
  std::string frame_name;

  unsigned int seed;
  double update_rate;
  double phinsbin_update_rate;
  double noisePR, noiseY, noiseVel, noiseAngVel, noiseAcc, noiseLat, noiseLon, noiseDep;
  bool use_gravity;

  double lat_origin_rad, lat_origin, lon_origin;

  // actual core data
  common::Time data_time;
  std::string entity_name;
  ignition::math::Quaterniond world2ll;
  ignition::math::Quaterniond orientation;
  ignition::math::Vector3d position;
  ignition::math::Vector3d angular_velocity;
  ignition::math::Vector3d linear_velocity;
  ignition::math::Vector3d body_linear_velocity;
  ignition::math::Vector3d linear_accel;
  double pitch, roll, heading;
  double latitude, longitude, altitude;
};

}; // namespace gazebo
