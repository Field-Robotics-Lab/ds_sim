//
// Created by ivaughn on 1/29/19.
//

#include "dsros_thruster.hh"
#include <geometry_msgs/WrenchStamped.h>

using namespace gazebo;

DsrosThruster::DsrosThruster() {
  command = 0;
}

void DsrosThruster::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  // Basic bookkeeping
  body_link = _parent->GetLink();

  GZ_ASSERT(_sdf->HasElement("pose"), "Thruster must define a pose");
  pose = _sdf->Get<ignition::math::Pose3d>("pose");

  GZ_ASSERT(_sdf->HasElement("thruster_link"), "Thruster must define a link name");
  thruster_link = _sdf->Get<std::string>("thruster_link");

  // load our thruster models
  GZ_ASSERT(_sdf->HasElement("forward"), "Thruster must define forward thrust coefficients");
  forward = LoadModel(_sdf->GetElement("forward"));

  if (_sdf->HasElement("reverse")) {
    reverse = LoadModel(_sdf->GetElement("reverse"));
  } else {
    gzwarn <<"Thruster plugin does not define reverse thrust coefficients; using forward ones instead" <<std::endl;
    reverse = LoadModel(_sdf->GetElement("forward"));
  }

  // prepare a unit vector along our axis of thrust
  thrust_unit_vec = pose.Rot().RotateVector(ignition::math::Vector3d::UnitX);

  // connect to ROS
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                  << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  node_handle.reset(new ros::NodeHandle);

  // prepare our topics
  GZ_ASSERT(_sdf->HasElement("cmd_topic"), "Thruster must define a command topic");
  std::string cmd_topic_name = _sdf->Get<std::string>("cmd_topic");
  cmd_sub = node_handle->subscribe(cmd_topic_name, 10, &DsrosThruster::OnCmdUpdate, this);

  GZ_ASSERT(_sdf->HasElement("viz_topic"), "Thruster must define a visualization output topic");
  std::string viz_topic_name = _sdf->Get<std::string>("viz_topic");
  viz_pub = node_handle->advertise<geometry_msgs::WrenchStamped>(viz_topic_name, 10);

  // register our OnUpdate callback
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DsrosThruster::OnUpdate, this, _1));
}

DsrosThrusterModel DsrosThruster::LoadModel(sdf::ElementPtr sdf) {
  GZ_ASSERT(sdf->HasElement("gain"), "Thruster model MUST have a gain");
  GZ_ASSERT(sdf->HasElement("offset"), "Thruster model MUST have an offset");

  DsrosThrusterModel ret;
  ret.gain = sdf->Get<double>("gain");
  ret.offset = sdf->Get<double>("offset");

  GZ_ASSERT(sdf->HasElement("speed_gain") == sdf->HasElement("speed_offset"),
      "Thrust model should either have speed_gain AND speed_offset or neither");

  if (sdf->HasElement("speed_gain")) {
    ret.speed_gain = sdf->Get<double>("speed_gain");
  } else {
    gzmsg <<"Thrust model does not specify a speed gain; assuming 0\n";
    ret.speed_gain = 0;
  }
  if (sdf->HasElement("speed_offset")) {
    ret.speed_offset = sdf->Get<double>("speed_offset");
  } else {
    gzmsg <<"Thrust model does not specify a speed offset; assuming 0\n";
    ret.speed_offset = 0;
  }

  return ret;
}

void DsrosThruster::OnCmdUpdate(const ds_actuator_msgs::ThrusterCmd& cmd) {
  if (body_link) {
    command = cmd.cmd_value;
    command_timeout = body_link->GetWorld()->GetSimTime() + common::Time(cmd.ttl_seconds);
  } else {
    command = 0;
    gzmsg <<"No model for thruster yet, forcing time to zero...\n";
  }
}

void DsrosThruster::OnUpdate(const common::UpdateInfo& _info) {
  // Check if our command has expire
  if (_info.simTime >= command_timeout) {
    if (command_timeout != common::Time(0)) {
      // only message on transition
      gzmsg << "Thruster " << thruster_link << " command has timed out.  Zeroing... \n";
    }
    command = 0;
    command_timeout = common::Time(0);
  }

  // Get our velocity in the direction of the thruster
  ignition::math::Vector3d body_vel = body_link->GetRelativeLinearVel().Ign();
  // we'll ignore the kinematic effects of rotation
  // In general, its NOT OK to do that.  But if rotation is a large component of our
  // apparant thruster velocity, then there's all sorts of other unmodelled effects
  // going on anyway... still.  TODO

  double apparant_velocity = body_vel.Dot(thrust_unit_vec);

  // if apparant velocity and command have opposite signs...
  if ( (apparant_velocity > 0 && command < 0) || (apparant_velocity < 0 && command > 0) ) {
    // clamp to avoid weird effects
    apparant_velocity = 0;

    // in reality, the thruster is LESS efficient in this regime, but we don't model that.
    // ... and its not super-important for ROVs nor super-common for AUVs
  }

  // calculate the scalar thrust
  double thrust_val = 0;
  if (command >= 0) {
    // this function will automatically clamp to 0 if the thrust is invalid
    thrust_val = forward.calc_thrust(command, thrust_val);
  } else {
    thrust_val = -reverse.calc_thrust(command, thrust_val);
  }

  // actually apply the force
  body_link->AddLinkForce(thrust_val*thrust_unit_vec, pose.Pos());

  // publish our wrench
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp.fromSec(_info.simTime.Float());
  msg.header.frame_id = thruster_link;
  msg.wrench.force.x = thrust_val;
  msg.wrench.force.y = 0;
  msg.wrench.force.z = 0;
  viz_pub.publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(DsrosThruster)