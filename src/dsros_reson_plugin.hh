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


#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ros/ros.h>
#include <string>
#include <ds_multibeam_msgs/MultibeamRaw.h>

namespace gazebo {

class dsrosRosResonSensor : public SensorPlugin {
 public:

  /// \brief Constructor
  dsrosRosResonSensor();

  /// \brief Destructor
  virtual ~dsrosRosResonSensor();

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
  ros::NodeHandle *node;

  /// \brief Reson publisher
  ros::Publisher reson_data_publisher;
  ros::Publisher pointcloud_publisher;

  ds_multibeam_msgs::MultibeamRaw mb_msg;
  sensor_msgs::PointCloud2 pc_msg;

  /// \brief last time on which the data was published.
  common::Time last_time;

  /// \brief Pointer to the update event connection.
  gazebo::event::ConnectionPtr connection;

  /// \brief Pointer to the sensor.
  std::shared_ptr<sensors::RaySensor> sensor;

  /// \brief Pointer to the sdf config file.
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string frame_name;

  unsigned int seed;
  double update_rate;
  double gaussian_noise;

  // actual core data
  // TODO
};

}