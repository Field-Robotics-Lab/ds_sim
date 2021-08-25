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
    rng_publisher = node->advertise<ds_sensor_msgs::Ranges3D>(ranges_topic_name, 1);
    pt_data_publisher  = node->advertise<sensor_msgs::PointCloud>(topic_name + "_cloud", 1);
    current_profile_publisher = node->advertise<ds_sensor_msgs::Adcp>(topic_name + "_current", 1);
    pt_data_publisher  = node->advertise<sensor_msgs::PointCloud>(topic_name + "_cloud", 1);
    connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                boost::bind(&dsrosRosDvlSensor::UpdateChild, this, _1));
    last_time = sensor->LastUpdateTime();

    // Initialize data members and the Adcp message for current profiling
    current_profile_cell_depth = (sensor->RangeMax() - sensor->RangeMin()) / water_track_bins;
    current_profile_bin0_distance = sensor->RangeMin() + current_profile_cell_depth / 2.0;
    for (size_t beam = 0; beam < 4; beam++)
    {
        ignition::math::Vector3d unit_vec = sensor->GetBeamUnitVec(beam);
        adcp.beam_unit_vec[beam].x = unit_vec.X();
        adcp.beam_unit_vec[beam].y = unit_vec.Y();
        adcp.beam_unit_vec[beam].z = unit_vec.Z();
    } // for (size_t beam = 0;...
    adcp.vel_bin_beams.resize(water_track_bins);
    for (size_t bin = 0; bin < water_track_bins; bin++)
    {
        if (current_profile_coord_mode == ds_sensor_msgs::Adcp::ADCP_COORD_BEAM)
        {
            adcp.vel_bin_beams[bin].velocity_bin_beam.resize(4);
//            adcp.vel_bin_beams[bin].bin_intensity.resize(4);     // Commented out in msg def for now
//            adcp.vel_bin_beams[bin].bin_correlation.resize(4);   // Commented out in msg def for now
        }
        else
        {
            adcp.vel_bin_beams[bin].velocity_bin_beam.resize(1);
//            adcp.vel_bin_beams[bin].bin_intensity.resize(1);     // Commented out in msg def for now
//            adcp.vel_bin_beams[bin].bin_correlation.resize(1);   // Commented out in msg def for now
        }
    } // for (size_t bin = 0;...
}

void dsrosRosDvlSensor::UpdateChild(const gazebo::common::UpdateInfo &_info) {

    common::Time current_time = sensor->LastUpdateTime();

    if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) {
        return;
    }

    // add noise and recompute the velocity
    std::vector<double> ranges(4);
    Eigen::VectorXd raw_beam_vel(4);
    Eigen::VectorXd beam_vel(4);
    Eigen::VectorXd beam_wtr_vel(4);
    Eigen::MatrixXd beam_unit(4,3);
    Eigen::MatrixXd beam_wtr_unit(4,3);
    Eigen::Vector3d velocity;
    Eigen::Vector3d wtr_velocity;
    double speed = 0;
    double course = 0;
    double altitude = 0;
    double wtr_speed = 0;
    double wtr_course = 0;
    int fillIn = 0;
    for (size_t i=0; i<sensor->NumBeams(); i++) {

        ignition::math::Vector3d beamUnit = sensor->GetBeamUnitVec(i);
        if (sensor->BeamValid(i)) {
            ranges[i] = sensor->GetBeamRange(i) + GaussianKernel(0, gaussian_noise_range);
            raw_beam_vel(i) = sensor->GetBeamVelocity(i) + GaussianKernel(0, gaussian_noise_vel);
            beam_vel(fillIn) = raw_beam_vel(i);
            beam_unit(fillIn, 0) = beamUnit.X();
            beam_unit(fillIn, 1) = beamUnit.Y();
            beam_unit(fillIn, 2) = beamUnit.Z();
            altitude += ranges[i];
            fillIn++;
        } else {
            ranges[i] = std::numeric_limits<double>::quiet_NaN();
        }
        beam_wtr_vel(i) = sensor->GetBeamWaterVelocity(i) + GaussianKernel(0, gaussian_noise_wtr_vel);
        beam_wtr_unit(i, 0) = beamUnit.X();
        beam_wtr_unit(i, 1) = beamUnit.Y();
        beam_wtr_unit(i, 2) = beamUnit.Z();
    }

    if (fillIn >= 3) {
        // trim the arrays
        beam_unit = beam_unit.topRows(fillIn);
        beam_vel = beam_vel.head(fillIn);

        // solve a least-squares problem to get velocity based on the noisy ranges
        velocity = beam_unit.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beam_vel);

        altitude /= static_cast<float>(fillIn);
        altitude *= cos(M_PI/6); // convert range to altitude

        speed = sqrt(velocity(0)*velocity(0) + velocity(1)*velocity(1));
        course = atan2(velocity(0), velocity(1)) * 180.0/M_PI;
        if (course < 0) {
            course += 360.0;
        }
    }
    // same method to compute a water track velocity based on the noisy beam water velocities
    wtr_velocity = beam_wtr_unit.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beam_wtr_vel);
    wtr_speed = sqrt(wtr_velocity(0)*wtr_velocity(0) + wtr_velocity(1)*wtr_velocity(1));
    wtr_course = atan2(wtr_velocity(0), wtr_velocity(1)) * 180.0/M_PI;
    if (wtr_course < 0) {
        wtr_course += 360.0;
    }

    // publish DVL data message
    if(dvl_data_publisher.getNumSubscribers() > 0) {

        // prepare message header
        msg.header.frame_id = frame_name;
        msg.header.stamp.sec = current_time.sec;
        msg.header.stamp.nsec = current_time.nsec;
        msg.header.seq++;

        msg.ds_header.io_time.sec = current_time.sec;
        msg.ds_header.io_time.nsec = current_time.nsec;

        if ((fillIn >= 3) || (!water_track_enabled)) {
            msg.velocity.x = velocity(0);
            msg.velocity.y = velocity(1);
            msg.velocity.z = velocity(2);
            msg.course_gnd = course;
            msg.speed_gnd = speed;
            msg.velocity_mode = ds_sensor_msgs::Dvl::DVL_MODE_BOTTOM;
            msg.velocity_covar[0] = gaussian_noise_vel*gaussian_noise_vel;
            msg.velocity_covar[1] = 0;
            msg.velocity_covar[2] = 0;

            msg.velocity_covar[3] = 0;
            msg.velocity_covar[4] = gaussian_noise_vel*gaussian_noise_vel;
            msg.velocity_covar[5] = 0;

            msg.velocity_covar[6] = 0;
            msg.velocity_covar[7] = 0;
            msg.velocity_covar[8] = gaussian_noise_vel*gaussian_noise_vel;
        } else {
            msg.velocity.x = wtr_velocity(0);
            msg.velocity.y = wtr_velocity(1);
            msg.velocity.z = wtr_velocity(2);
            msg.course_gnd = wtr_course;
            msg.speed_gnd = wtr_speed;
            msg.velocity_mode = ds_sensor_msgs::Dvl::DVL_MODE_WATER;
            msg.velocity_covar[0] = gaussian_noise_wtr_vel*gaussian_noise_wtr_vel;
            msg.velocity_covar[1] = 0;
            msg.velocity_covar[2] = 0;

            msg.velocity_covar[3] = 0;
            msg.velocity_covar[4] = gaussian_noise_wtr_vel*gaussian_noise_wtr_vel;
            msg.velocity_covar[5] = 0;

            msg.velocity_covar[6] = 0;
            msg.velocity_covar[7] = 0;
            msg.velocity_covar[8] = gaussian_noise_wtr_vel*gaussian_noise_wtr_vel;
        }
        msg.num_good_beams = sensor->ValidBeams();
        msg.speed_sound = 1500.0;
        msg.altitude = altitude;

        for (size_t i=0; i<sensor->NumBeams(); i++) {
            msg.range[i] = ranges[i];
            msg.range_covar[i] = gaussian_noise_range*gaussian_noise_range;
            ignition::math::Vector3d beamUnit = sensor->GetBeamUnitVec(i);
            msg.beam_unit_vec[i].x = beamUnit.X();
            msg.beam_unit_vec[i].y = beamUnit.Y();
            msg.beam_unit_vec[i].z = beamUnit.Z();
            if (fillIn >= 3) {
                msg.raw_velocity[i] = raw_beam_vel(i);
                msg.raw_velocity_covar[i] = gaussian_noise_vel*gaussian_noise_vel;
            } else {
                msg.raw_velocity[i] = beam_wtr_vel(i);
                msg.raw_velocity_covar[i] = gaussian_noise_wtr_vel*gaussian_noise_wtr_vel;
            }
        }

        //ROS_INFO_STREAM("DVL_SENDING_INST: " <<velocity(0) <<" " <<velocity(1) <<" " <<velocity(2));

        msg.coordinate_mode = ds_sensor_msgs::Dvl::DVL_COORD_INSTRUMENT;
        msg.dvl_time = static_cast<double>(current_time.sec) + static_cast<double>(current_time.nsec)/1.0e9;

        // publish data
        dvl_data_publisher.publish(msg);
        ros::spinOnce();
    }

    // publish current profile (ADCP) message
    if ((current_profile_publisher.getNumSubscribers() > 0) && 
        ((current_profile_coord_mode == ds_sensor_msgs::Adcp::ADCP_COORD_BEAM) ||
         (current_profile_coord_mode == ds_sensor_msgs::Adcp::ADCP_COORD_INSTRUMENT)) &&
        (water_track_enabled)) {  // No current profile if water track unavailable or unsupported mode
        // prepare message header
        adcp.header.frame_id = frame_name;
        adcp.header.stamp.sec = current_time.sec;
        adcp.header.stamp.nsec = current_time.nsec;
        adcp.header.seq++;

        msg.ds_header.io_time.sec = current_time.sec;
        msg.ds_header.io_time.nsec = current_time.nsec;

        // Fill message-level fields
        adcp.coordinate_mode = current_profile_coord_mode;
        adcp.adcp_type = ds_sensor_msgs::Adcp::ADCP_TYPE_PISTON;
        adcp.cells = water_track_bins;
        adcp.cell_depth = current_profile_cell_depth;
        adcp.bin0_distance = current_profile_bin0_distance;

        // Fill in beam-specific bin velocities as water velocity plus noise
        // out to the beam's current range.  No solution (0.0) beyond that
        for (size_t bin = 0; bin < water_track_bins; bin ++)
        {
            double bin_range = current_profile_bin0_distance + 
                               (current_profile_cell_depth * bin);

            if (current_profile_coord_mode == ds_sensor_msgs::Adcp::ADCP_COORD_BEAM)
            {   // Calculate a velocity in beam coordinates for each beam for every cell
                for (size_t beam=0; beam < sensor->NumBeams(); beam++)
                {
                    // add noise to the beam's bin-specific velocity
                    double bin_velocity = sensor->GetBeamWaterVelocityBin(beam, bin) +
                                          GaussianKernel(0, gaussian_noise_wtr_vel);
                    if (bin_velocity != gazebo::sensors::DsrosDvlBeam::NO_VELOCITY)
                    {
                        ignition::math::Vector3d beamUnit = sensor->GetBeamUnitVec(beam);
                        adcp.vel_bin_beams[bin].velocity_bin_beam[beam].x = -bin_velocity * beamUnit.X();
                        adcp.vel_bin_beams[bin].velocity_bin_beam[beam].y = -bin_velocity * beamUnit.Y();
                        adcp.vel_bin_beams[bin].velocity_bin_beam[beam].z = -bin_velocity * beamUnit.Z();
                    } // if (beam_range >=...
                    else 
                    {
                        adcp.vel_bin_beams[bin].velocity_bin_beam[beam].x = 0.0;
                        adcp.vel_bin_beams[bin].velocity_bin_beam[beam].y = 0.0;
                        adcp.vel_bin_beams[bin].velocity_bin_beam[beam].z = 0.0;
                    } // else
                } // for (size_t beam = 0;...
            } // if (current_profile_coord_mode ==...
            else
            {   // Calculate a single velocity in instrument coordinates for every cell
                bool range_solution = true;
                for (size_t beam=0; beam < sensor->NumBeams(); beam++)
                {
                    double bin_velocity = sensor->GetBeamWaterVelocityBin(beam, bin);
                    if (range_solution &&
                        (bin_velocity != gazebo::sensors::DsrosDvlBeam::NO_VELOCITY))
                    {
                        ignition::math::Vector3d beamUnit = sensor->GetBeamUnitVec(beam);
                        beam_wtr_vel(beam) = sensor->GetBeamWaterVelocityBin(beam, bin) +
                                             GaussianKernel(0, gaussian_noise_wtr_vel);
                        beam_wtr_unit(beam, 0) = beamUnit.X();
                        beam_wtr_unit(beam, 1) = beamUnit.Y();
                        beam_wtr_unit(beam, 2) = beamUnit.Z();
                    } // if (!no_solution...
                    else
                    {
                        range_solution = false;
                    } // else
                } // for (size_t beam=0;...
                if (range_solution)  // if bin for any beam is beyond bottom, solution will be 0
                {
                    Eigen::Vector3d bin_velocity =
                        beam_wtr_unit.jacobiSvd(Eigen::ComputeThinU |
                                                Eigen::ComputeThinV).
                                      solve(beam_wtr_vel);
                    adcp.vel_bin_beams[bin].velocity_bin_beam[0].x = -bin_velocity[0];
                    adcp.vel_bin_beams[bin].velocity_bin_beam[0].y = -bin_velocity[1];
                    adcp.vel_bin_beams[bin].velocity_bin_beam[0].z = -bin_velocity[2];
                } // if (range_solution)
                else
                {
                    adcp.vel_bin_beams[bin].velocity_bin_beam[0].x = 0.0;
                    adcp.vel_bin_beams[bin].velocity_bin_beam[0].y = 0.0;
                    adcp.vel_bin_beams[bin].velocity_bin_beam[0].z = 0.0;
                } // else
            } // else
        } // for (size_t bin = 0;...

        current_profile_publisher.publish(adcp);
        ros::spinOnce();
    }

    // publish point cloud message
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

    // publish DVL contact range message
    if (rng_publisher.getNumSubscribers() > 0) {
        // prepare message header
        rng.header.frame_id = frame_name;
        rng.header.stamp.sec = current_time.sec;
        rng.header.stamp.nsec = current_time.nsec;
        rng.header.seq++;

        // fill in some points
        size_t NUM_PTS_PER_BEAM = 1000;
        rng.ranges.resize(sensor->NumBeams());
        for (size_t j=0; j<sensor->NumBeams(); j++) {
            // correctly report ranges in the instrument frame
            ignition::math::Vector3d beamUnit = sensor->GetBeamUnitVec(j);
      	    rng.ranges[j].range.point.x = ranges[j]*beamUnit.X();
      	    rng.ranges[j].range.point.y = ranges[j]*beamUnit.Y();
      	    rng.ranges[j].range.point.z = ranges[j]*beamUnit.Z();
      	    if (sensor->BeamValid(j)) {
                rng.ranges[j].range_validity = ds_sensor_msgs::Range3D::RANGE_VALID;
                rng.ranges[j].range_quality = 250;
            } else {
                rng.ranges[j].range_validity = ds_sensor_msgs::Range3D::RANGE_INDETERMINANT;
                rng.ranges[j].range_quality = 10;
            }
    	      rng.ranges[j].range.header.frame_id = frame_name;
	          rng.ranges[j].range.header.stamp.sec = current_time.sec;
      	    rng.ranges[j].range.header.stamp.nsec = current_time.nsec;
        }
        // publish data
        rng_publisher.publish(rng);
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

  //RANGES TOPIC
  if (sdf->HasElement("rangesTopicName"))
  {
    ranges_topic_name =  sdf->Get<std::string>("rangesTopicName");
    ROS_INFO_STREAM("<rangesTopicName> set to: "<<ranges_topic_name);
  }
  else
  {
    ranges_topic_name = "/dvl_ranges";
    ROS_WARN_STREAM("missing <rangesTopicName>, set to /namespace/default: " << ranges_topic_name);
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

  if (sdf->HasElement("gaussianNoiseBeamWtrVel"))
  {
    gaussian_noise_wtr_vel =  sdf->Get<double>("gaussianNoiseBeamWtrVel");
    ROS_INFO_STREAM("<gaussianNoiseBeamWtrVel> set to: " << gaussian_noise_wtr_vel);
  }
  else
  {
    gaussian_noise_wtr_vel = 2.0 * gaussian_noise_vel;
    ROS_WARN_STREAM("missing <gaussianNoiseBeamWtrVel>, set to default: " << gaussian_noise_wtr_vel);
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

  //WATER TRACKING
  if (sdf->HasElement("enableWaterTrack"))
  {
    water_track_enabled = sdf->Get<bool>("enableWaterTrack");
    ROS_INFO_STREAM("<enableWaterTrack> set to: " << water_track_enabled);
  }
  else
  {
    water_track_enabled = false;
    ROS_WARN_STREAM("missing <enableWaterTrack>, set to default: " << water_track_enabled);
  }

  if (sdf->HasElement("currentProfileCoordMode") && water_track_enabled)
  {
    current_profile_coord_mode = sdf->Get<int>("currentProfileCoordMode");
    ROS_INFO_STREAM("<currentProfileCoordMode> set to: " << current_profile_coord_mode);
  }
  else if (water_track_enabled)
  {
    current_profile_coord_mode = ds_sensor_msgs::Adcp::ADCP_COORD_BEAM;
    ROS_WARN_STREAM("missing <currentProfileCoordMode>, set to default: " << current_profile_coord_mode);
  }

  if (sdf->HasElement("waterTrackBins") && water_track_enabled)
  {
    water_track_bins = sdf->Get<int>("waterTrackBins");
    ROS_INFO_STREAM("<waterTrackBins> set to: " << water_track_bins);
  }
  else if (water_track_enabled)
  {
    water_track_bins = 1;
    ROS_WARN_STREAM("missing <waterTrackBins>, set to default: " << water_track_bins);
  }

  return true;
}
