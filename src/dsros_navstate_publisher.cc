//
// Created by ivaughn on 2/2/18.
//

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

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
      gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
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

    enable_tf_broadcast = true;
    if (_sdf->HasElement("enable_tf")) {
      enable_tf_broadcast = _sdf->Get<bool>("enable_tf");
    }


    // Connect to the ROS node
    ROS_INFO_STREAM("dsros_navstate_publisher: Initializing internal ROS node...");
    rosNode.reset(new ros::NodeHandle(robotNamespace));

    // prepare some reference times
    double updateRate = 10;
    if (_sdf->HasElement("updateRate")) {
      updateRate = _sdf->Get<double>("updateRate");
    }
    rosPublishPeriod = gazebo::common::Time(1.0/updateRate);
    lastRosPublished = gazebo::common::Time(0.0);

    // Setup our publisher
    ROS_INFO_STREAM("dsros_navstate_publisher: Advertising topic to publish on...");
    navstatePub = rosNode->advertise<ds_nav_msgs::AggregatedState>(navstate_topic, 10);
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
    math::Pose vehPose = model->GetWorldPose();
    math::Vector3 vehWorldVel = model->GetWorldLinearVel();
    math::Vector3 vehAngVel = model->GetWorldAngularVel();
    // recover body-frame velocities
    math::Vector3 vehLinVel = vehPose.rot.RotateVectorReverse(vehWorldVel);

    // publish a navigation state message
    ds_nav_msgs::AggregatedState state_msg;

    state_msg.header.stamp = now;
    state_msg.header.frame_id = frame_id;
    state_msg.ds_header.io_time = now;

    // convert to NED
    state_msg.northing.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.northing.value = vehPose.pos.y;
    state_msg.easting.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.easting.value = vehPose.pos.x;
    state_msg.down.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.down.value = -vehPose.pos.z;

    // convert yaw -> heading
    state_msg.roll.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.roll.value = vehPose.rot.GetRoll();
    state_msg.pitch.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.pitch.value = -vehPose.rot.GetPitch();
    state_msg.heading.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.heading.value = M_PI/2 - vehPose.rot.GetYaw();

    // convert to fwd/stbd/down
    state_msg.surge_u.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.surge_u.value = vehLinVel.x;
    state_msg.sway_v.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.sway_v.value = -vehLinVel.y;
    state_msg.heave_w.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.heave_w.value = -vehLinVel.z;

    // convert yaw -> heading
    // Not exactly sure which rotation axes gazebo uses
    state_msg.p.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.p.value = vehAngVel.x;
    state_msg.q.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.q.value = -vehAngVel.y;
    state_msg.r.valid = ds_nav_msgs::FlaggedDouble::VALUE_VALID;
    state_msg.r.value = -vehAngVel.z;

    navstatePub.publish(state_msg);

    // publish a pose for comparison in rviz
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.position.x = vehPose.pos.x;
    pose_msg.pose.position.y = vehPose.pos.y;
    pose_msg.pose.position.z = vehPose.pos.z;
    pose_msg.pose.orientation.x = vehPose.rot.x;
    pose_msg.pose.orientation.y = vehPose.rot.y;
    pose_msg.pose.orientation.z = vehPose.rot.z;
    pose_msg.pose.orientation.w = vehPose.rot.w;

    posePub.publish(pose_msg);

    // publish a TF
    if (enable_tf_broadcast) {
      tf::Transform tform;
      tform.setOrigin(tf::Vector3(vehPose.pos.x, vehPose.pos.y, vehPose.pos.z));
      tform.setRotation(tf::Quaternion(vehPose.rot.x, vehPose.rot.y, vehPose.rot.z, vehPose.rot.w));
      tform_broadcaster.sendTransform(tf::StampedTransform(tform, now, frame_id, base_link_name));
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
  std::string frame_id;
  std::string base_link_name;
};

GZ_REGISTER_MODEL_PLUGIN(DsNavStatePublisher);

}