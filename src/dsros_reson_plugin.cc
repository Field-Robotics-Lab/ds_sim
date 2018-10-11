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


#include "dsros_reson_plugin.hh"


using namespace gazebo;

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

    // TODO: Add publishers for pointcloud
    reson_data_publisher = node->advertise<ds_multibeam_msgs::MultibeamRaw>(topic_name + "/raw_multibeam", 5);

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

    // TODO: Remove this
    // Quick demo to make sure we're actually getting real ranges.
    int centerbeam_idx = sensor->RangeCount() / 2;
    std::cout <<"Center beam: " <<sensor->Range(centerbeam_idx) <<std::endl;

    if(reson_data_publisher.getNumSubscribers() > 0) {

        // TODO: Fill in mb_msg

        // publish data
        reson_data_publisher.publish(mb_msg);
        ros::spinOnce();
    }

    // TODO: Add a publisher for the pointcloud data

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