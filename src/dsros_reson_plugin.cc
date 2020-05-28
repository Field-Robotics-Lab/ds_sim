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


#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "dsros_reson_plugin.hh"


using namespace gazebo;

constexpr static const double RTOD = 180.0/M_PI;

GZ_REGISTER_SENSOR_PLUGIN(dsrosRosResonSensor);

dsrosRosResonSensor::dsrosRosResonSensor() : SensorPlugin() {
    sensor = NULL;
    seed = 0;
};

dsrosRosResonSensor::~dsrosRosResonSensor() {
    if (connection.get()) {
        connection.reset();
    }
    node->shutdown();
}

void dsrosRosResonSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_) {
    sdf = sdf_;
    sensor = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor_);
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

    reson_data_publisher = node->advertise<ds_multibeam_msgs::MultibeamRaw>(topic_name + "/mbraw", 5);
    pointcloud_publisher = node->advertise<sensor_msgs::PointCloud2>(topic_name + "/pointcloud", 10);

  // connect our UpdateChild function to get called whenever there's a world update event
    connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                boost::bind(&dsrosRosResonSensor::UpdateChild, this, _1));
    last_time = sensor->LastUpdateTime();
}

void dsrosRosResonSensor::UpdateChild(const gazebo::common::UpdateInfo &_info) {

    common::Time current_time = sensor->LastUpdateTime();

    if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) {
        return;
    }

    int num_beams = sensor->RangeCount();

    if(reson_data_publisher.getNumSubscribers() > 0 || pointcloud_publisher.getNumSubscribers() > 0) {

        mb_msg.header.frame_id = frame_name;
        mb_msg.header.stamp.sec = current_time.sec;
        mb_msg.header.stamp.nsec = current_time.nsec;
        mb_msg.ds_header.io_time = mb_msg.header.stamp;
        mb_msg.soundspeed = 1500;

        mb_msg.beamflag.resize(num_beams);
        mb_msg.intensity.resize(num_beams);
        mb_msg.angleAcrossTrack.resize(num_beams);
        mb_msg.angleAlongTrack.resize(num_beams);
        mb_msg.twowayTravelTime.resize(num_beams);
        mb_msg.txDelay.resize(num_beams);

        pc_msg.header = mb_msg.header;
        pc_msg.height = 1;
        pc_msg.width = num_beams;
        sensor_msgs::PointCloud2Modifier modifier(pc_msg);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                    sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(num_beams);
        sensor_msgs::PointCloud2Iterator<float> iter_distance(pc_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_acrossTrack(pc_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_alongTrack(pc_msg, "z");

        double angle = sensor->AngleMin().Radian();
        double angle_increment = sensor->AngleResolution();
        for (size_t i=0; i<num_beams; i++) {
            double range = sensor->Range(i);

            mb_msg.beamflag[i] = ds_multibeam_msgs::MultibeamRaw::BEAM_OK;
            mb_msg.intensity[i] = 0; // not supported by ROS.  Yay.
            mb_msg.angleAcrossTrack[i] = angle;
            mb_msg.angleAlongTrack[i] = 0;
            mb_msg.twowayTravelTime[i] = 2.0*range / 1500.0;
            mb_msg.txDelay[i] = 0;

            *iter_acrossTrack = sin(angle) * range;
            *iter_distance = cos(angle) * range;
            *iter_alongTrack = 0;

            angle += angle_increment;
            ++iter_distance;
            ++iter_acrossTrack;
            ++iter_alongTrack;
        }

      // publish data
        reson_data_publisher.publish(mb_msg);

        pointcloud_publisher.publish(pc_msg);
        ros::spinOnce();
    }

    last_time = current_time;
}

double dsrosRosResonSensor::GaussianKernel(double mu, double sigma) {
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool dsrosRosResonSensor::LoadParameters() {

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
    topic_name = "/depth";
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