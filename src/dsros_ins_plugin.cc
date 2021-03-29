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
#include "dsros_ins_plugin.hh"

using namespace gazebo;

constexpr static const double RTOD = 180.0/M_PI;

GZ_REGISTER_SENSOR_PLUGIN(dsrosRosInsSensor);

dsrosRosInsSensor::dsrosRosInsSensor() : SensorPlugin(), world2ll(M_PI, 0, M_PI/2.0) {
    roll = 0;
    pitch = 0;
    heading = 0;

    sensor = NULL;
    seed = 0;
};

dsrosRosInsSensor::~dsrosRosInsSensor() {
    if (connection.get()) {
        connection.reset();
    }
    node->shutdown();
}

void dsrosRosInsSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_) {
    sdf = sdf_;
    sensor = std::dynamic_pointer_cast<gazebo::sensors::DsrosInsSensor>(sensor_);
    if (sensor == NULL) {
        ROS_FATAL("Error! Unable to convert sensor pointer!");
        return;
    }
    sensor->SetActive(true);

    if (!LoadParameters()) {
        ROS_FATAL("Error loading parameters for sensor plugin!");
        return;
    }

    // Grab our lat/lon origin
    auto world = gazebo::physics::get_world(sensor->WorldName());
#if GAZEBO_MAJOR_VERSION > 7
    auto spherical = world->SphericalCoords();
#else
    auto spherical = world->GetSphericalCoordinates();
#endif
    lat_origin_rad = spherical->LatitudeReference().Radian();
    lat_origin = spherical->LatitudeReference().Degree();
    lon_origin = spherical->LongitudeReference().Degree();

    // setup our sensor
    sensor->SetAddGravity(use_gravity);

    if (!ros::isInitialized()) {
      ROS_FATAL("ROS has not been initialized properly...");
      return;
    }

    node = new ros::NodeHandle(this->robot_namespace);

    ins_publisher = node->advertise<ds_sensor_msgs::Ins>(ins_topic_name, 1);
    gyro_publisher = node->advertise<ds_sensor_msgs::Gyro>(gyro_topic_name, 1);
    att_publisher = node->advertise<geometry_msgs::QuaternionStamped>(att_topic_name, 1);
    if (!phinsbin_topic_name.empty()) {
      phinsbin_publisher = node->advertise<ds_sensor_msgs::PhinsStdbin3>(phinsbin_topic_name, 10);
    }
    connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                boost::bind(&dsrosRosInsSensor::UpdateChild, this, _1));
    last_time = sensor->LastUpdateTime();
  last_phinsbin_time = sensor->LastUpdateTime();
}

void dsrosRosInsSensor::UpdateChild(const gazebo::common::UpdateInfo &_info) {

  common::Time current_time = sensor->LastUpdateTime();

  // if we aren't going to be doing any messages, return now
  if((update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate)
      && (phinsbin_update_rate > 0 && (current_time - last_phinsbin_time).Double() < 1.0/phinsbin_update_rate)) {
    return;
  }

  // if ANYONE is listening, update our sensors & add noise
  if( (!phinsbin_topic_name.empty() && phinsbin_publisher.getNumSubscribers() > 0) || ins_publisher.getNumSubscribers() > 0
      || gyro_publisher.getNumSubscribers() > 0 || att_publisher.getNumSubscribers() > 0) {

    // update our raw data
    data_time = sensor->GetTime();
    entity_name = sensor->GetEntityName();
    orientation = sensor->GetOrientation();
    angular_velocity = sensor->GetAngularVelocity();
    linear_velocity = sensor->GetLinearVelocity();
    linear_accel = sensor->GetLinearAcceleration();
    latitude = sensor->GetLatitude();
    longitude = sensor->GetLongitude();
    altitude = sensor->GetAltitude();
    position = sensor->GetPosition();

    // add noise
    // In order to make the noise correct, define a tiny little
    // perturbation to the rotation measurement
    ignition::math::Quaterniond noiseTform(GaussianKernel(0, noisePR),
                                           GaussianKernel(0, noisePR),
                                           GaussianKernel(0, noiseY));
    // ... and then apply it to the measurement
    orientation = noiseTform*orientation;

    angular_velocity.X() += GaussianKernel(0, noiseAngVel);
    angular_velocity.Y() += GaussianKernel(0, noiseAngVel);
    angular_velocity.Z() += GaussianKernel(0, noiseAngVel);

    linear_velocity.X() += GaussianKernel(0, noiseVel);
    linear_velocity.Y() += GaussianKernel(0, noiseVel);
    linear_velocity.Z() += GaussianKernel(0, noiseVel);

    body_linear_velocity = orientation.RotateVectorReverse(linear_velocity);

    linear_accel.X() += GaussianKernel(0, noiseAcc);
    linear_accel.Y() += GaussianKernel(0, noiseAcc);
    linear_accel.Z() += GaussianKernel(0, noiseAcc);

    double noise_north = GaussianKernel(0, noiseLat);
    double noise_east = GaussianKernel(0, noiseLon);
    double noise_alt = GaussianKernel(0, noiseDep);

    // The spherical coordinates implementation is kinda broken, so use ours
    // terrible AlvinXY approximation instead
    double latrad = lat_origin_rad;
    // use the mdeglat/mdeglon stuff from dslpp
    double dx = 111415.13 * cos(latrad) - 94.55 * cos(3.0*latrad)
              + 0.12 * cos(5.0*latrad);
    double dy = 111132.09 - 566.05 * cos(2.0*latrad) + 1.20 * cos(4.0*latrad)
              - 0.002 * cos(6.0*latrad);
    longitude = lon_origin + (position.X() + noise_east)/dx;
    latitude  = lat_origin + (position.Y() + noise_north)/dy;
    altitude = position.Z() + noise_alt;
  }

  if (phinsbin_update_rate > 0 && (current_time - last_phinsbin_time).Double() >= 1.0/phinsbin_update_rate) {
    // do the phinsbin update
    if (!phinsbin_topic_name.empty() && phinsbin_publisher.getNumSubscribers() > 0) {
      // standard header stuff
      phinsbin_msg.header.stamp.sec = data_time.sec;
      phinsbin_msg.header.stamp.nsec = data_time.nsec;
      phinsbin_msg.header.frame_id = frame_name;
      phinsbin_msg.nav_validity_time = static_cast<double>(data_time.sec) + static_cast<double>(data_time.nsec)*1.0e-9;
      phinsbin_msg.counter++;

      // heading/pitch/roll + rates + stddev
      phinsbin_msg.roll = orientation.Roll() * RTOD;
      phinsbin_msg.pitch = orientation.Pitch() * RTOD;
      phinsbin_msg.heading = M_PI/2 - orientation.Yaw();
      if (phinsbin_msg.heading > 2*M_PI) {
        phinsbin_msg.heading -= (2*M_PI);
      } else if (phinsbin_msg.heading < 0.0) {
        phinsbin_msg.heading += (2 * M_PI);
      }
      phinsbin_msg.heading *= RTOD;

      phinsbin_msg.roll_rate = angular_velocity.X()*RTOD;
      phinsbin_msg.pitch_rate = angular_velocity.Y()*RTOD;
      phinsbin_msg.heading_rate = angular_velocity.Z()*RTOD;

      phinsbin_msg.heading_stddev = noiseY*RTOD;
      phinsbin_msg.roll_stddev = noisePR*RTOD;
      phinsbin_msg.pitch_stddev = noisePR*RTOD;

      // now orientation-- NOTE-- this requires a little transform from ENU to NWU
      ignition::math::Quaterniond nwu2enu(sqrt(2.0)/2.0, 0, 0, sqrt(2.0)/2.0);
      ignition::math::Quaterniond phins_quat = orientation.Inverse() * nwu2enu;
      phinsbin_msg.attitude_quaternion[0] = phins_quat.W();
      phinsbin_msg.attitude_quaternion[1] = phins_quat.X();
      phinsbin_msg.attitude_quaternion[2] = phins_quat.Y();
      phinsbin_msg.attitude_quaternion[3] = phins_quat.Z();

      phinsbin_msg.attitude_quaternion_stddev[0] = noisePR;
      phinsbin_msg.attitude_quaternion_stddev[1] = noisePR;
      phinsbin_msg.attitude_quaternion_stddev[2] = noiseY;

      // position
      phinsbin_msg.longitude = longitude;
      phinsbin_msg.latitude = latitude;
      phinsbin_msg.altitude = altitude; // depth is actually altitude

      // body velocity
      phinsbin_msg.body_velocity_XVn[0] = body_linear_velocity.X();
      phinsbin_msg.body_velocity_XVn[1] = body_linear_velocity.Y();
      phinsbin_msg.body_velocity_XVn[2] = body_linear_velocity.Z();

      // global velocity noise
      phinsbin_msg.velocity_NEU[0] = linear_velocity.X();
      phinsbin_msg.velocity_NEU[1] = linear_velocity.Y();
      phinsbin_msg.velocity_NEU[2] = linear_velocity.Z();

      // global velocity noise
      phinsbin_msg.velocity_stddev_NEU[0] = noiseVel;
      phinsbin_msg.velocity_stddev_NEU[1] = noiseVel;
      phinsbin_msg.velocity_stddev_NEU[2] = noiseVel;

      // now body-rates
      phinsbin_msg.body_rates_XVn[0] = angular_velocity.X(); // XV1-- roll
      phinsbin_msg.body_rates_XVn[1] = angular_velocity.Y(); // XV2-- pitch
      phinsbin_msg.body_rates_XVn[2] = angular_velocity.Z(); // XV3-- yaw

      phinsbin_msg.body_rotrate_stddev_XVn[0] = noiseAngVel; // XV1-- roll
      phinsbin_msg.body_rotrate_stddev_XVn[1] = noiseAngVel; // XV2-- pitch
      phinsbin_msg.body_rotrate_stddev_XVn[2] = noiseAngVel; // XV3-- yaw

      // finally, accelerations
      phinsbin_msg.body_accel_XVn[0] = linear_accel.X(); // XV1-- fwd
      phinsbin_msg.body_accel_XVn[1] = linear_accel.Y(); // XV2-- port
      phinsbin_msg.body_accel_XVn[2] = linear_accel.Z(); // XV3-- up

      phinsbin_msg.body_accel_stddev_XVn[0] = noiseAcc;
      phinsbin_msg.body_accel_stddev_XVn[1] = noiseAcc;
      phinsbin_msg.body_accel_stddev_XVn[2] = noiseAcc;

      phinsbin_publisher.publish(phinsbin_msg);
      ros::spinOnce();
    }

    last_phinsbin_time = current_time;
  }

  // if none of the low-rate stuff is going to happen, just return here
  if (update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) {
    return;
  }


  if(ins_publisher.getNumSubscribers() > 0 || gyro_publisher.getNumSubscribers() > 0) {

    // prepare message header
    ins_msg.header.frame_id = frame_name;
    ins_msg.header.stamp.sec = data_time.sec;
    ins_msg.header.stamp.nsec = data_time.nsec;
    ins_msg.header.seq++;

    ins_msg.ds_header.io_time.sec = data_time.sec;
    ins_msg.ds_header.io_time.nsec = data_time.nsec;

    ins_msg.orientation.x = orientation.X();
    ins_msg.orientation.y = orientation.Y();
    ins_msg.orientation.z = orientation.Z();
    ins_msg.orientation.w = orientation.W();

    ins_msg.linear_velocity.x = linear_velocity.X();
    ins_msg.linear_velocity.y = linear_velocity.Y();
    ins_msg.linear_velocity.z = linear_velocity.Z();

    ins_msg.angular_velocity.x = angular_velocity.X();
    ins_msg.angular_velocity.y = angular_velocity.Y();
    ins_msg.angular_velocity.z = angular_velocity.Z();

    ins_msg.linear_acceleration.x = linear_accel.X();
    ins_msg.linear_acceleration.y = linear_accel.Y();
    ins_msg.linear_acceleration.z = linear_accel.Z();

    ins_msg.roll = orientation.Roll();
    ins_msg.pitch = -orientation.Pitch();
    ins_msg.heading = M_PI/2 - orientation.Yaw();
    if (ins_msg.heading > 2*M_PI) {
      ins_msg.heading -= (2*M_PI);
    } else if (ins_msg.heading < 0.0) {
      ins_msg.heading += (2*M_PI);
    }
  }

  if(ins_publisher.getNumSubscribers() > 0) {

    ins_msg.latitude = latitude;

    // publish data
    ins_publisher.publish(ins_msg);
    ros::spinOnce();
  }

  if (gyro_publisher.getNumSubscribers() > 0) {
    gyro_msg.header = ins_msg.header;
    gyro_msg.ds_header = ins_msg.ds_header;

    gyro_msg.heading = ins_msg.heading;
    gyro_msg.pitch = ins_msg.pitch;
    gyro_msg.roll = ins_msg.roll;

    gyro_msg.heading_covar = noiseY;
    gyro_msg.pitch_covar = noisePR;
    gyro_msg.roll_covar = noisePR;

    gyro_msg.orientation = ins_msg.orientation;

    gyro_publisher.publish(gyro_msg);
  }

  if (att_publisher.getNumSubscribers() > 0) {
    att_msg.header.frame_id = frame_name;
    att_msg.header.stamp.sec = data_time.sec;
    att_msg.header.stamp.nsec = data_time.nsec;
    att_msg.header.seq++;

    att_msg.quaternion.x = orientation.X();
    att_msg.quaternion.y = orientation.Y();
    att_msg.quaternion.z = orientation.Z();
    att_msg.quaternion.w = orientation.W();

    att_publisher.publish(att_msg);
    ros::spinOnce();
  }

  last_time = current_time;
}

double dsrosRosInsSensor::GaussianKernel(double mu, double sigma) {
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool dsrosRosInsSensor::LoadParameters() {

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

  use_gravity = true;
  if (sdf->HasElement("includeGravity")) {
    use_gravity = sdf->Get<bool>("includeGravity");
  }
  ROS_INFO_STREAM("<includeGravity> set to: " <<use_gravity);

  //TOPICS
  if (sdf->HasElement("insTopicName"))
  {
    ins_topic_name =  sdf->Get<std::string>("insTopicName");
    ROS_INFO_STREAM("<insTopicName> set to: "<<ins_topic_name);
  }
  else
  {
    ins_topic_name = "/ins";
    ROS_WARN_STREAM("missing <insTopicName>, set to /namespace/default: " << ins_topic_name);
  }

  if (sdf->HasElement("gyroTopicName"))
  {
    gyro_topic_name =  sdf->Get<std::string>("gyroTopicName");
    ROS_INFO_STREAM("<gyroTopicName> set to: "<<gyro_topic_name);
  }
  else
  {
    gyro_topic_name = "/gyro";
    ROS_WARN_STREAM("missing <gyroTopicName>, set to /namespace/default: " << gyro_topic_name);
  }

  if (sdf->HasElement("attTopicName"))
  {
    att_topic_name =  sdf->Get<std::string>("attTopicName");
    ROS_INFO_STREAM("<attTopicName> set to: "<<att_topic_name);
  }
  else
  {
    ins_topic_name = "/attitude";
    ROS_WARN_STREAM("missing <attTopicName>, set to /namespace/default: " << att_topic_name);
  }

  if (sdf->HasElement("phinsbinTopicName"))
  {
    phinsbin_topic_name =  sdf->Get<std::string>("phinsbinTopicName");
    ROS_INFO_STREAM("<attTopicName> set to: "<<phinsbin_topic_name);
  }
  else
  {
    phinsbin_topic_name = "/phinsbin";
    ROS_WARN_STREAM("missing <phinsbinTopicName>, set to /namespace/default: " << phinsbin_topic_name);
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

  //PHINSBIN UPDATE RATE
  if (sdf->HasElement("phinsbinUpdateRateHZ"))
  {
    phinsbin_update_rate =  sdf->Get<double>("phinsbinUpdateRateHZ");
    ROS_INFO_STREAM("<phinsbinUpdateRateHZ> set to: " << phinsbin_update_rate);
  }
  else
  {
    phinsbin_update_rate = update_rate;
    ROS_WARN_STREAM("missing <phinsbinUpdateRateHZ>, set to default: " << phinsbin_update_rate);
  }

  noisePR     = LoadNoise("gaussianNoisePR", M_PI/180.0);
  noiseY      = LoadNoise("gaussianNoiseY", M_PI/180.0);
  noiseVel    = LoadNoise("gaussianNoiseVel", 1.0);
  noiseAngVel = LoadNoise("gaussianNoiseAngVel", M_PI/180.0);
  noiseAcc    = LoadNoise("gaussianNoiseAcc", 1.0);
  noiseLat    = LoadNoise("gaussianNoiseLat", 1.0);
  noiseLon    = LoadNoise("gaussianNoiseLon", 1.0);
  noiseDep    = LoadNoise("gaussianNoiseDepth", 1.0);

  return true;
}

double dsrosRosInsSensor::LoadNoise(const std::string& tag, double units) const {

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
