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
#include <gazebo/physics/physics.hh>
#include "dsros_gps_plugin.hh"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(dsrosRosGpsSensor);

dsrosRosGpsSensor::dsrosRosGpsSensor() : SensorPlugin() {

    sensor = NULL;
    seed = 0;
};

dsrosRosGpsSensor::~dsrosRosGpsSensor() {
    if (connection.get()) {
        connection.reset();
    }
    node->shutdown();
}

void dsrosRosGpsSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_) {
    sdf = sdf_;
    sensor = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(sensor_);
    if (sensor == NULL) {
        ROS_FATAL("Error! Unable to convert sensor pointer!");
        return;
    }
    sensor->SetActive(true);

    // Gazebo's Spherical Coordinates conversion is b0rk3n.  We have to do our own.
    // Start by grabbing the origin
    auto world = gazebo::physics::get_world(sensor->WorldName());
#if GAZEBO_MAJOR_VERSION > 7
    auto spherical = world->SphericalCoords();
#else
  auto spherical = world->GetSphericalCoordinates();
#endif
    lat_origin_rad = spherical->LatitudeReference().Radian();
    lat_origin = spherical->LatitudeReference().Degree();
    lon_origin = spherical->LongitudeReference().Degree();
    gzmsg <<"ORIGIN: " <<lat_origin <<", " <<lon_origin <<std::endl;

#if GAZEBO_MAJOR_VERSION > 7
    physics::EntityPtr parentEntity = world->EntityByName(sensor->ParentName());
#else
    physics::EntityPtr parentEntity = world->GetEntity(sensor->ParentName());
#endif
    parent_link = boost::dynamic_pointer_cast<gazebo::physics::Link>(parentEntity);

  if (!LoadParameters()) {
        ROS_FATAL("Error loading parameters for sensor plugin!");
        return;
    }

    if (!ros::isInitialized()) {
        ROS_FATAL("ROS has not been initialized properly...");
        return;
    }

    node = new ros::NodeHandle(this->robot_namespace);

    publisher = node->advertise<sensor_msgs::NavSatFix>(topic_name, 1);
    connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                boost::bind(&dsrosRosGpsSensor::UpdateChild, this, _1));
    last_time = sensor->LastUpdateTime();
}

void dsrosRosGpsSensor::UpdateChild(const gazebo::common::UpdateInfo &_info) {

    common::Time current_time = sensor->LastUpdateTime();

    if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) {
        return;
    }

    if(publisher.getNumSubscribers() > 0) {


        // update our raw data
        data_time = sensor->LastMeasurementTime();
        entity_name = frame_name;
        // Gazebo is ENU.  Unless you use the Spherical Coordinates module, then it's screwy
#if GAZEBO_MAJOR_VERSION > 7
        ignition::math::Pose3d gpsPose = sensor->Pose() + parent_link->WorldPose();
#else
        ignition::math::Pose3d gpsPose = sensor->Pose() + parent_link->GetWorldPose().Ign();
#endif
        double easting = gpsPose.Pos().X();
        double northing = gpsPose.Pos().Y();
        altitude  = sensor->Altitude();

        double latrad = lat_origin_rad;
        // use the mdeglat/mdeglon stuff from dslpp
        double dx = 111415.13 * cos(latrad) - 94.55 * cos(3.0*latrad)
            		+ 0.12 * cos(5.0*latrad);
        double dy = 111132.09 - 566.05 * cos(2.0*latrad) + 1.20 * cos(4.0*latrad)
                    - 0.002 * cos(6.0*latrad);


        // We use local AlvinXY for this.  Its terrible, but we're using REALLY small
        // displacements, so its valid most places
        latitude  = lat_origin + (northing + GaussianKernel(0, noiseLL_m))/dy;
        longitude = lon_origin + (easting + GaussianKernel(0, noiseLL_m))/dx;
        altitude  += GaussianKernel(0, noiseZ_m);

        gps_msg.header.frame_id = frame_name;
        gps_msg.header.stamp.sec = data_time.sec;
        gps_msg.header.stamp.nsec = data_time.nsec;
        gps_msg.header.seq++;

        // Only fill in valid data if we're above the minimum altitude
        // (0 is the sea surface, so no GPS underwater!)
        if (sensor->Altitude() > min_altitude) {
            gps_msg.latitude = latitude;
            gps_msg.longitude = longitude;
            gps_msg.altitude = altitude;
            gps_msg.status.status = gps_msg.status.STATUS_FIX;
            gps_msg.status.service = gps_msg.status.SERVICE_GPS;

            gps_msg.position_covariance[0] = noiseLL_m * noiseLL_m;
            gps_msg.position_covariance[1] = 0;
            gps_msg.position_covariance[2] = 0;

            gps_msg.position_covariance[3] = 0;
            gps_msg.position_covariance[4] = noiseLL_m * noiseLL_m;
            gps_msg.position_covariance[5] = 0;

            gps_msg.position_covariance[6] = 0;
            gps_msg.position_covariance[7] = 0;
            gps_msg.position_covariance[8] = noiseZ_m * noiseZ_m;

            gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
        } else {
            // if we're underwater, return no data
            gps_msg.latitude = 0;
            gps_msg.longitude = 0;
            gps_msg.altitude = 0;
            gps_msg.status.status = gps_msg.status.STATUS_NO_FIX;
            gps_msg.status.service = gps_msg.status.SERVICE_GPS;

            gps_msg.position_covariance[0] = 0;
            gps_msg.position_covariance[1] = 0;
            gps_msg.position_covariance[2] = 0;

            gps_msg.position_covariance[3] = 0;
            gps_msg.position_covariance[4] = 0;
            gps_msg.position_covariance[5] = 0;

            gps_msg.position_covariance[6] = 0;
            gps_msg.position_covariance[7] = 0;
            gps_msg.position_covariance[8] = 0;

            gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_UNKNOWN;
        }

        publisher.publish(gps_msg);
        ros::spinOnce();
    }

    last_time = current_time;
}

double dsrosRosGpsSensor::GaussianKernel(double mu, double sigma) {
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool dsrosRosGpsSensor::LoadParameters() {

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

  //TOPICS
  if (sdf->HasElement("topicName"))
  {
    topic_name =  sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: "<<topic_name);
  }
  else
  {
    topic_name = "/gps";
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

    // MINIMUM_ALTITUDE
    if (sdf->HasElement("minAltitude")) {
        min_altitude = sdf->Get<double>("minAltitude");
        ROS_INFO_STREAM("<minAltitude> set to: " << min_altitude);
    } else {
        min_altitude = 0.0;
        ROS_INFO_STREAM("missing <minAltitude> set to default: " << min_altitude);
    }


    // noise states
  noiseLL_m   = LoadNoise("gaussianNoiseLL_meters", 1);
  noiseZ_m    = LoadNoise("gaussianNoiseAlt_meters", 1);

  return true;
}

double dsrosRosGpsSensor::LoadNoise(const std::string& tag, double units) const {

  double gaussian_noise = 0;

  if (sdf->HasElement(tag))
  {
    gaussian_noise =  sdf->Get<double>(tag);
    ROS_INFO_STREAM("<" <<tag <<"> set to : " << gaussian_noise);
    gaussian_noise *= units;
  }
  else
  {
    ROS_WARN_STREAM("missing <" <<tag <<">, set to default: " << gaussian_noise);
  }

  return gaussian_noise;
}
