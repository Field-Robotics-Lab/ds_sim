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
//
// Created by ivaughn on 2/2/18.
//

// REV: MELODIC 1 0

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#if GAZEBO_MAJOR_VERSION > 7
#include <gazebo/common/CommonIface.hh>
#else
#include <gazebo/common/common.hh>
#endif

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <ds_nav_msgs/AggregatedState.h>

namespace gazebo {

class DsNavStatePublisher : public ModelPlugin {
 public:
  virtual ~DsNavStatePublisher() {
    if (updateConnection) {
#if GAZEBO_MAJOR_VERSION > 7
      // Melodic API change
      // Deprecation: public: void Events::Disconnect.*(ConnectionPtr);
      // Replacement: Delete the Connection object, perhaps by calling reset() on its smart pointer.
      this->updateConnection.reset();
#else
      gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
    }

    if (rosNode) {
      rosNode->shutdown();
    }
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    // Check on ROS
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                           << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Get the proper robot name
    this->model = _parent;
    if (_sdf->HasElement("robotNamespace")) {
      this->robotNamespace = _sdf->Get<std::string>("robotNamespace");
    } else {
      this->robotNamespace = this->model->GetName();
    }

    // Get the topic name
    std::string navstate_topic = "nav/agg/state";
    if (_sdf->HasElement("navstate_topic")) {
      navstate_topic = _sdf->Get<std::string>("navstate_topic");
    }

    std::string pose_topic = "sim/vehicle/true_pose";
    if (_sdf->HasElement("simpose_topic")) {
      pose_topic = _sdf->Get<std::string>("simpose_topic");
    }

    // And the frame we're publishing in
    frame_id = "odom";
    if (_sdf->HasElement("frame_id")) {
      frame_id = _sdf->Get<std::string>("frame_id");
    }

    base_link_name = "base_link";
    if (_sdf->HasElement("base_link")) {
      base_link_name = _sdf->Get<std::string>("base_link");
    }

    // Connect to the ROS node
    ROS_INFO_STREAM("dsros_navstate_publisher: Initializing internal ROS node...");
    rosNode.reset(new ros::NodeHandle(robotNamespace));


    // Read the ROS BROADCAST flag
    enable_tf_broadcast = true;
    if (_sdf->HasElement("enable_tf")) {
      enable_tf_broadcast = _sdf->Get<bool>("enable_tf");
    }
    if (rosNode->hasParam("sim/enable_tf_broadcast")) {
      rosNode->getParam("sim/enable_tf_broadcast", enable_tf_broadcast);
    } else {
      ROS_WARN_STREAM("Using XACRO for dsros_navstate_publisher::enable_tf_broadcast."
                          <<" To change, try setting rosparam "
                          <<rosNode->resolveName("enable_tf_broadcast"));
    }
    // read the TF Frame to use if enable_tf_broadcast is off
    tf_frame_name = frame_id;
    if (_sdf->HasElement("tf_frame_name")) {
      tf_frame_name = _sdf->Get<std::string>("tf_frame_name");
    }
    if (rosNode->hasParam("sim/tf_frame_name")) {
      rosNode->getParam("sim/tf_frame_name", tf_frame_name);
    }

    // read the navstate broadcast enable flag
    enable_navstate_broadcast = true;
    if (_sdf->HasElement("enable_navstate")) {
      enable_navstate_broadcast = _sdf->Get<bool>("enable_navstate");
    }
    if (rosNode->hasParam("sim/enable_navstate_broadcast")) {
      rosNode->getParam("sim/enable_navstate_broadcast", enable_navstate_broadcast);
    } else {
      ROS_WARN_STREAM("Using XACRO for dsros_navstate_publisher::enable_navstate_broadcast."
                          <<" To change, try setting rosparam "
                          <<rosNode->resolveName("enable_navstate_broadcast"));
    }

    // prepare some reference times
    double updateRate = 10;
    if (_sdf->HasElement("updateRate")) {
      updateRate = _sdf->Get<double>("updateRate");
    }
    rosPublishPeriod = gazebo::common::Time(1.0/updateRate);
    lastRosPublished = gazebo::common::Time(0.0);

    // Setup our publisher
    ROS_INFO_STREAM("dsros_navstate_publisher: Advertising topic to publish on...");
    if (enable_navstate_broadcast) {
      navstatePub = rosNode->advertise<ds_nav_msgs::AggregatedState>(navstate_topic, 10);
    }
    posePub = rosNode->advertise<geometry_msgs::PoseStamped>(pose_topic, 10);

    // Listen to the update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DsNavStatePublisher::OnUpdate, this, _1));
  }

  void OnUpdate(const common::UpdateInfo& _info) {

    if (_info.simTime < lastRosPublished + rosPublishPeriod) {
      return;
    }

    // NOW!
    ros::Time now = ros::Time::now();
#if GAZEBO_MAJOR_VERSION > 7
    ignition::math::Pose3d vehPose = model->WorldPose();
    ignition::math::Vector3d vehWorldVel = model->WorldLinearVel();
    ignition::math::Vector3d vehAngVel = model->WorldAngularVel();
#else
    ignition::math::Pose3d vehPose = model->GetWorldPose().Ign();
    ignition::math::Vector3d vehWorldVel = model->GetWorldLinearVel().Ign();
    ignition::math::Vector3d vehAngVel = model->GetWorldAngularVel().Ign();
#endif
    ignition::math::Vector3d vehLinVel = vehPose.Rot().RotateVectorReverse(vehWorldVel);

    // publish a navigation state message
    ds_nav_msgs::AggregatedState state_msg;

    state_msg.header.stamp = now;
    state_msg.header.frame_id = frame_id;
    state_msg.ds_header.io_time = now;

    // convert to NED
    state_msg.northing.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.easting.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.down.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.northing.value = vehPose.Pos().Y();
    state_msg.easting.value = vehPose.Pos().X();
    state_msg.down.value = -vehPose.Pos().Z();

    // convert yaw -> heading
    state_msg.roll.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.pitch.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.heading.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.roll.value = vehPose.Rot().Roll();
    state_msg.pitch.value = -vehPose.Rot().Pitch();
    state_msg.heading.value = M_PI/2 - vehPose.Rot().Yaw();

    // convert to fwd/stbd/down
    state_msg.surge_u.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.sway_v.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.heave_w.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.surge_u.value = vehLinVel.X();
    state_msg.sway_v.value = -vehLinVel.Y();
    state_msg.heave_w.value = -vehLinVel.Z();

    // convert yaw -> heading
    // Not exactly sure which rotation axes gazebo uses
    state_msg.p.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.q.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.r.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.p.value = vehAngVel.X();
    state_msg.q.value = -vehAngVel.Y();
    state_msg.r.value = -vehAngVel.Z();

    if (enable_navstate_broadcast) {
      navstatePub.publish(state_msg);
    }

    // publish a pose for comparison in rviz
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.position.x = vehPose.Pos().X();
    pose_msg.pose.position.y = vehPose.Pos().Y();
    pose_msg.pose.position.z = vehPose.Pos().Z();
    pose_msg.pose.orientation.x = vehPose.Rot().X();
    pose_msg.pose.orientation.y = vehPose.Rot().Y();
    pose_msg.pose.orientation.z = vehPose.Rot().Z();
    pose_msg.pose.orientation.w = vehPose.Rot().W();

    posePub.publish(pose_msg);

    // publish a TF
    tf::Transform tform;
    tform.setOrigin(tf::Vector3(vehPose.Pos().X(), vehPose.Pos().Y(), vehPose.Pos().Z()));
    tform.setRotation(tf::Quaternion(vehPose.Rot().X(), vehPose.Rot().Y(), vehPose.Rot().Z(), vehPose.Rot().W()));
    if (enable_tf_broadcast) {
      tform_broadcaster.sendTransform(tf::StampedTransform(tform, now, frame_id, base_link_name));
    } else {
      // if we're not broadcasting directly, STILL send a tf but use a different name and re-order it so that
      // there is a single unambiguous transform root
      tform_broadcaster.sendTransform(tf::StampedTransform(tform.inverse(), now, base_link_name, tf_frame_name));
    }

    // update housekeeping data
    lastRosPublished += rosPublishPeriod;
  }

 protected:
  event::ConnectionPtr updateConnection;
  std::string robotNamespace;
  physics::ModelPtr model;

  gazebo::common::Time rosPublishPeriod, lastRosPublished;

  std::unique_ptr<ros::NodeHandle> rosNode;

  ros::Publisher navstatePub;
  ros::Publisher posePub;
  tf::TransformBroadcaster tform_broadcaster;

  bool enable_tf_broadcast;
  bool enable_navstate_broadcast;
  std::string frame_id;
  std::string tf_frame_name;
  std::string base_link_name;
};

GZ_REGISTER_MODEL_PLUGIN(DsNavStatePublisher);

}
