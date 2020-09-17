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
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <ds_sensor_msgs/Dvl.h>
#include <ds_sensor_msgs/Ranges3D.h>
#include <ds_sensor_msgs/Adcp.h>
#include <ds_sensor_msgs/WaterCurrentProfile.h>
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

  /// \brief current profile publisher
  ros::Publisher current_profile_publisher;

  ds_sensor_msgs::Dvl msg;
  ds_sensor_msgs::Ranges3D rng;
  ds_sensor_msgs::Adcp adcp;


  // temporary stuff to make sure we're point our beams correctly
  ros::Publisher pt_data_publisher;
  ros::Publisher rng_publisher;
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
  std::string ranges_topic_name;
  std::string frame_name;

  unsigned int seed;
  double update_rate;
  double gaussian_noise_vel;
  double gaussian_noise_wtr_vel;
  double gaussian_noise_range;
  bool water_track_enabled;
  int water_track_bins;
  int current_profile_coord_mode;
  double current_profile_cell_depth;
  double current_profile_bin0_distance;
};

}; // namespace gazebo
