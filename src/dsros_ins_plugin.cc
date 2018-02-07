// lots of this implementation blatently stolen from:
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp
#include "dsros_ins_plugin.hh"

using namespace gazebo;

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

    if (!ros::isInitialized()) {
        ROS_FATAL("ROS has not been initialized properly...");
        return;
    }

    node = new ros::NodeHandle(this->robot_namespace);

    ins_publisher = node->advertise<ds_sensor_msgs::Ins>(ins_topic_name, 1);
    att_publisher = node->advertise<geometry_msgs::QuaternionStamped>(att_topic_name, 1);
    connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                boost::bind(&dsrosRosInsSensor::UpdateChild, this, _1));
    last_time = sensor->LastUpdateTime();
}

void dsrosRosInsSensor::UpdateChild(const gazebo::common::UpdateInfo &_info) {

    common::Time current_time = sensor->LastUpdateTime();

    if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) {
        return;
    }

    if(ins_publisher.getNumSubscribers() > 0
                || att_publisher.getNumSubscribers() > 0) {

        // update our raw data
        data_time = sensor->GetTime();
        entity_name = sensor->GetEntityName();
        orientation = sensor->GetOrientation();
        angular_velocity = sensor->GetAngularVelocity();
        linear_velocity = sensor->GetLinearVelocity();
        linear_accel = sensor->GetLinearAcceleration();
        latitude = sensor->GetLatitude();

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

        linear_accel.X() += GaussianKernel(0, noiseAcc);
        linear_accel.Y() += GaussianKernel(0, noiseAcc);
        linear_accel.Z() += GaussianKernel(0, noiseAcc);

        latitude += GaussianKernel(0, noiseLat);
    }

    if(ins_publisher.getNumSubscribers() > 0) {

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

        // use a local-level frame to get rotations correct
        ignition::math::Quaterniond ll_rot = orientation;
        ins_msg.roll = orientation.Roll()*180.0/M_PI;
        ins_msg.pitch = -orientation.Pitch()*180.0/M_PI;
        ins_msg.heading = 90-orientation.Yaw()*180.0/M_PI;
        if (ins_msg.heading > 360.0) {
            ins_msg.heading -= 360.0;
        } else if (ins_msg.heading < 0.0) {
            ins_msg.heading += 360.0;
        }

        ins_msg.latitude = latitude;

        // publish data
        ins_publisher.publish(ins_msg);
        ros::spinOnce();
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

  noisePR     = LoadNoise("gaussianNoisePR", M_PI/180.0);
  noiseY      = LoadNoise("gaussianNoiseY", M_PI/180.0);
  noiseVel    = LoadNoise("gaussianNoiseVel", 1.0);
  noiseAngVel = LoadNoise("gaussianNoiseAngVel", M_PI/180.0);
  noiseAcc    = LoadNoise("gaussianNoiseAcc", 1.0);
  noiseLat    = LoadNoise("gaussianNoiseLat", 1.0);

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
