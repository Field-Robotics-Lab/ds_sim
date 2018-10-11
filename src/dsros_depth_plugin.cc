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

// lots of this implementation blatently stolen from:
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp
#include "dsros_depth_plugin.hh"
#include "../gazebo_src/depth_util.cc" // yes, really include the .cpp

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(dsrosRosDepthSensor);

dsrosRosDepthSensor::dsrosRosDepthSensor() : SensorPlugin() {
    depth = 0;
    pressure = 0;
    latitude = 0;

    sensor = NULL;
    seed = 0;
};

dsrosRosDepthSensor::~dsrosRosDepthSensor() {
    if (connection.get()) {
        connection.reset();
    }
    node->shutdown();
}

void dsrosRosDepthSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_) {
    sdf = sdf_;
    sensor = std::dynamic_pointer_cast<gazebo::sensors::DsrosDepthSensor>(sensor_);
    if (sensor == NULL) {
        ROS_FATAL("Error! Unable to convert sensor pointer!");
        return;
    }
    sensor->SetActive(true);

    if (!LoadParameters()) {
        ROS_FATAL("Error loading parameters for sensor plugin!");
        return;
    }

    if (!ros::isInitialized()) {
        ROS_FATAL("ROS has not been initialized properly...");
        return;
    }

    node = new ros::NodeHandle(this->robot_namespace);

    depth_data_publisher = node->advertise<ds_sensor_msgs::DepthPressure>(topic_name, 1);
    connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                boost::bind(&dsrosRosDepthSensor::UpdateChild, this, _1));
    last_time = sensor->LastUpdateTime();
}

void dsrosRosDepthSensor::UpdateChild(const gazebo::common::UpdateInfo &_info) {

    common::Time current_time = sensor->LastUpdateTime();

    if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) {
        return;
    }

    if(depth_data_publisher.getNumSubscribers() > 0) {

        pressure = sensor->GetPressure();
        latitude = sensor->GetLatitude();

        // prepare message header
        msg.header.frame_id = frame_name;
        msg.header.stamp.sec = current_time.sec;
        msg.header.stamp.nsec = current_time.nsec;
        msg.header.seq++;

        msg.ds_header.io_time.sec = current_time.sec;
        msg.ds_header.io_time.nsec = current_time.nsec;

        msg.pressure = pressure + GaussianKernel(0, gaussian_noise);
        msg.pressure_raw_unit = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_DBAR;

        msg.tare = 10.1325; // 1 atm
        msg.pressure_raw = msg.pressure + msg.tare;
        msg.latitude = latitude;
        msg.depth = fofonoff_depth(msg.pressure, msg.latitude);
        msg.pressure_covar = gaussian_noise * gaussian_noise;

	// Simulator should really run the median filter too
	msg.median_depth_filter_ok = true;
	
        // publish data
        depth_data_publisher.publish(msg);
        ros::spinOnce();
    }

    last_time = current_time;
}

double dsrosRosDepthSensor::GaussianKernel(double mu, double sigma) {
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool dsrosRosDepthSensor::LoadParameters() {

//loading parameters from the sdf file

  //NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace =  sdf->Get<std::string>("robotNamespace") +"/";
    ROS_INFO_STREAM("<robotNamespace> set to: "<<robot_namespace);
  }
  else
  {
    std::string scoped_name = sensor->ParentName();
    std::size_t it = scoped_name.find("::");

    robot_namespace = "/" +scoped_name.substr(0,it)+"/";
    ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
  }

  //TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name =  sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: "<<topic_name);
  }
  else
  {
    topic_name = "/reson";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  //BODY NAME
  if (sdf->HasElement("frameName"))
  {
    frame_name =  sdf->Get<std::string>("frameName");
    ROS_INFO_STREAM("<frameName> set to: "<<frame_name);
  }
  else
  {
    ROS_FATAL("missing <frameName>, cannot proceed");
    return false;
  }

  //UPDATE RATE
  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate =  sdf->Get<double>("updateRateHZ");
    ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
  }
  else
  {
    update_rate = 1.0;
    ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
  }

  //NOISE
  if (sdf->HasElement("gaussianNoise"))
  {
    gaussian_noise =  sdf->Get<double>("gaussianNoise");
    ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  }
  else
  {
    gaussian_noise = 0.0;
    ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  }
  return true;
}
