
// lots of this implementation blatently stolen from:
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp
#include "dsros_dvl_plugin.hh"

#include <Eigen/Core>
#include <Eigen/SVD>

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(dsrosRosDvlSensor);

dsrosRosDvlSensor::dsrosRosDvlSensor() : SensorPlugin() {

    sensor = NULL;
    seed = 0;
};

dsrosRosDvlSensor::~dsrosRosDvlSensor() {
    if (connection.get()) {
        connection.reset();
    }
    node->shutdown();
}

void dsrosRosDvlSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_) {
    sdf = sdf_;
    sensor = std::dynamic_pointer_cast<gazebo::sensors::DsrosDvlSensor>(sensor_);
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

    dvl_data_publisher = node->advertise<ds_sensor_msgs::Dvl>(topic_name, 1);
    pt_data_publisher  = node->advertise<sensor_msgs::PointCloud>(topic_name + "_cloud", 1);
    connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                boost::bind(&dsrosRosDvlSensor::UpdateChild, this, _1));
    last_time = sensor->LastUpdateTime();
}

void dsrosRosDvlSensor::UpdateChild(const gazebo::common::UpdateInfo &_info) {

    common::Time current_time = sensor->LastUpdateTime();

    if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) {
        return;
    }

    // add noise and recompute the velocity
    std::vector<double> ranges(4);
    Eigen::VectorXd beam_vel(4);
    Eigen::MatrixXd beam_unit(4,3);
    Eigen::Vector3d velocity;
    int fillIn = 0;
    for (size_t i=0; i<sensor->NumBeams(); i++) {

        if (sensor->BeamValid(i)) {
            ignition::math::Vector3d beamUnit = sensor->GetBeamUnitVec(i);
            ranges[i] = sensor->GetBeamRange(i) + GaussianKernel(0, gaussian_noise_range);
            beam_vel(fillIn) = sensor->GetBeamVelocity(i) + GaussianKernel(0, gaussian_noise_vel);
            beam_unit(fillIn, 0) = beamUnit.X();
            beam_unit(fillIn, 1) = beamUnit.Y();
            beam_unit(fillIn, 2) = beamUnit.Z();
            fillIn++;
        } else {
            ranges[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    if (fillIn >= 3) {
        // trim the arrays
        beam_unit = beam_unit.topRows(fillIn);
        beam_vel = beam_vel.head(fillIn);

        // solve a least-squares problem to get velocity based on the noisy ranges
        velocity = beam_unit.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beam_vel);
    }



    if(dvl_data_publisher.getNumSubscribers() > 0) {

        // prepare message header
        msg.header.frame_id = frame_name;
        msg.header.stamp.sec = current_time.sec;
        msg.header.stamp.nsec = current_time.nsec;
        msg.header.seq++;

        msg.ds_header.io_time.sec = current_time.sec;
        msg.ds_header.io_time.nsec = current_time.nsec;

        msg.velocity.x = velocity(0);
        msg.velocity.y = velocity(1);
        msg.velocity.z = velocity(2);

        msg.velocity_covar[0] = gaussian_noise_vel*gaussian_noise_vel;
        msg.velocity_covar[1] = 0;
        msg.velocity_covar[2] = 0;

        msg.velocity_covar[3] = 0;
        msg.velocity_covar[4] = gaussian_noise_vel*gaussian_noise_vel;
        msg.velocity_covar[5] = 0;

        msg.velocity_covar[6] = 0;
        msg.velocity_covar[7] = 0;
        msg.velocity_covar[8] = gaussian_noise_vel*gaussian_noise_vel;

        msg.num_good_beams = sensor->ValidBeams();
        msg.speed_sound = 1500.0;

        for (size_t i=0; i<sensor->NumBeams(); i++) {
            msg.range[i] = ranges[i];
            msg.range_covar[i] = gaussian_noise_range*gaussian_noise_range;
            ignition::math::Vector3d beamUnit = sensor->GetBeamUnitVec(i);
            msg.beam_unit_vec[i].x = beamUnit.X();
            msg.beam_unit_vec[i].y = beamUnit.Y();
            msg.beam_unit_vec[i].z = beamUnit.Z();
        }

        ROS_INFO_STREAM("DVL_SENDING_INST: " <<velocity(0) <<" " <<velocity(1) <<" " <<velocity(2));

        msg.velocity_mode = ds_sensor_msgs::Dvl::DVL_MODE_BOTTOM;
        msg.coordinate_mode = ds_sensor_msgs::Dvl::DVL_COORD_INSTRUMENT;
        msg.dvl_time = static_cast<double>(current_time.sec) + static_cast<double>(current_time.nsec)/1.0e9;

        // publish data
        dvl_data_publisher.publish(msg);
        ros::spinOnce();
    }

    if (pt_data_publisher.getNumSubscribers() > 0) {
        // prepare message header
        pt_msg.header.frame_id = pointcloud_frame;
        pt_msg.header.stamp.sec = current_time.sec;
        pt_msg.header.stamp.nsec = current_time.nsec;
        pt_msg.header.seq++;

        // fill in some points
        size_t NUM_PTS_PER_BEAM = 1000;
        double range = sensor->RangeMax() - sensor->RangeMin();
        pt_msg.points.resize(NUM_PTS_PER_BEAM*sensor->NumBeams()); // use 100 pts
        size_t fillIn = 0;
        for (size_t j=0; j<sensor->NumBeams(); j++) {
            ignition::math::Pose3d beamPose = sensor->GetBeamPose(j);
            for (size_t i=0; i<NUM_PTS_PER_BEAM; i++) {
                ignition::math::Vector3d vec;
                vec.X() = 0;
                vec.Y() = 0;
                vec.Z() = sensor->RangeMin() + static_cast<double>(i)/static_cast<double>(
                                NUM_PTS_PER_BEAM-1)*(range);
    
                ignition::math::Vector3d beam = beamPose.Rot().RotateVector(vec) + beamPose.Pos();
                pt_msg.points[fillIn].x = beam.X();
                pt_msg.points[fillIn].y = beam.Y();
                pt_msg.points[fillIn].z = beam.Z();
                fillIn++;
            }
        }
        // publish data
        pt_data_publisher.publish(pt_msg);
        ros::spinOnce();
    }


    last_time = current_time;
}

double dsrosRosDvlSensor::GaussianKernel(double mu, double sigma) {
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool dsrosRosDvlSensor::LoadParameters() {

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
    topic_name = "/dvl";
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

  if (sdf->HasElement("pointcloudFrame")) {
    pointcloud_frame =  sdf->Get<std::string>("pointcloudFrame");
    ROS_INFO_STREAM("<pointcloudFrame> set to: "<<pointcloud_frame);
  }
  else
  {
    ROS_FATAL("missing <pointcloudFrame>, cannot proceed");
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
  if (sdf->HasElement("gaussianNoiseBeamVel"))
  {
    gaussian_noise_vel =  sdf->Get<double>("gaussianNoiseBeamVel");
    ROS_INFO_STREAM("<gaussianNoiseBeamVel> set to: " << gaussian_noise_vel);
  }
  else
  {
    gaussian_noise_vel = 0.0;
    ROS_WARN_STREAM("missing <gaussianNoiseBeamVel>, set to default: " << gaussian_noise_vel);
  }

  if (sdf->HasElement("gaussianNoiseBeamRange"))
  {
    gaussian_noise_range =  sdf->Get<double>("gaussianNoiseBeamRange");
    ROS_INFO_STREAM("<gaussianNoiseBeamRange> set to: " << gaussian_noise_range);
  }
  else
  {
    gaussian_noise_range = 0.0;
    ROS_WARN_STREAM("missing <gaussianNoiseBeamRange>, set to default: " << gaussian_noise_range);
  }
  return true;
}
