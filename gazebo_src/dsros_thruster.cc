//
// Created by ivaughn on 1/29/19.
//

#include "dsros_thruster.hh"
#include <geometry_msgs/WrenchStamped.h>
#include <ds_actuator_msgs/ThrusterState.h>

using namespace gazebo;

DsrosThruster::DsrosThruster() {
  command = 0;
  vehicle_in_loop = false;
  enabled = true;
  posIsFwd = false;
}

void DsrosThruster::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  // Basic bookkeeping
  body_link = _parent->GetLink();
  thruster_name = _parent->GetName();

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

  // check if this thruster is flipped in the driver
  if (_sdf->HasElement("propdir")) {
    int tmp=_sdf->Get<int>("propdir");
    std::cout <<"Thruster Link: " <<thruster_link <<" dir: " <<tmp <<"\n";
    if (tmp > 0) {
      posIsFwd = false;
    } else {
      posIsFwd = true;
    }
  }

  // Check if we're running a hardware-in-the-loop sim, or something else
  GZ_ASSERT(_sdf->HasElement("sim_param_namespace"), "Thruster must have a per-vehicle simulation parameter namespace");
  std::string sim_param_namespace = _sdf->Get<std::string>("sim_param_namespace");

  vehicle_in_loop = ros::param::param<bool>(sim_param_namespace + "/vehicle/robot_in_loop", false);

  // prepare our topics
  GZ_ASSERT(_sdf->HasElement("cmd_topic"), "Thruster must define a command topic");
  std::string cmd_topic_name = _sdf->Get<std::string>("cmd_topic");
  cmd_sub = node_handle->subscribe(cmd_topic_name, 10, &DsrosThruster::OnCmdUpdate, this);


  GZ_ASSERT(_sdf->HasElement("state_topic"), "Thruster must define a command topic");
  std::string state_topic_name = _sdf->Get<std::string>("state_topic");
  if (!vehicle_in_loop) {
    // if the vehicle is in the loop, the actual thruster driver should publish state
    state_pub = node_handle->advertise<ds_actuator_msgs::ThrusterState>(state_topic_name, 10);
  }

  GZ_ASSERT(_sdf->HasElement("viz_topic"), "Thruster must define a visualization topic");
  std::string viz_topic_name = _sdf->Get<std::string>("viz_topic");
  viz_pub = node_handle->advertise<geometry_msgs::WrenchStamped>(viz_topic_name, 10);

  // We need our update rate too
  GZ_ASSERT(_sdf->HasElement("updateRate"), "Thruster must define how often to publish messages, in Hz");
  double updateRate = _sdf->Get<double>("updateRate");
  publish_period = common::Time(1.0/updateRate);

  // register our OnUpdate callback
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DsrosThruster::OnUpdate, this, _1));
}

DsrosThrusterModel DsrosThruster::LoadModel(sdf::ElementPtr sdf) {
  GZ_ASSERT(sdf->HasElement("gain"), "Thruster model MUST have a gain");
  GZ_ASSERT(sdf->HasElement("offset"), "Thruster model MUST have an offset");
  GZ_ASSERT(sdf->HasElement("max_cmd"), "Thruster model MUST have a maximum command");

  DsrosThrusterModel ret;
  ret.gain = sdf->Get<double>("gain");
  ret.offset = sdf->Get<double>("offset");
  ret.max_cmd = sdf->Get<double>("max_cmd");

  if (sdf->HasElement("quad_gain")) {
    ret.quad_gain = sdf->Get<double>("quad_gain");
  } else {
    gzmsg <<"Thrust model does not specify a quadratic gain; assuming 0\n";
    ret.quad_gain = 0;
  }

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
    // flipping commands at the driver only affects reported outputs
    command = cmd.cmd_value;
#if GAZEBO_MAJOR_VERSION > 7
    command_timeout = body_link->GetWorld()->SimTime() + common::Time(cmd.ttl_seconds);
#else
    command_timeout = body_link->GetWorld()->GetSimTime() + common::Time(cmd.ttl_seconds);
#endif
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
  if (!enabled) {
    command = 0;
    command_timeout = common::Time(0);
  }

  // Get our velocity in the direction of the thruster
#if GAZEBO_MAJOR_VERSION > 7
  ignition::math::Vector3d body_vel = body_link->RelativeLinearVel();
#else
  ignition::math::Vector3d body_vel = body_link->GetRelativeLinearVel().Ign();
#endif
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
  bool thrust_forward = command >= 0;
  if (posIsFwd) {
    thrust_forward = !thrust_forward;
  }
  if (thrust_forward) {
    // this function will automatically clamp to 0 if the thrust is invalid
    thrust_val = forward.calc_thrust(fabs(command), fabs(apparant_velocity));
  } else {
    thrust_val = -reverse.calc_thrust(fabs(command), fabs(apparant_velocity));
  }

  // actually apply the force
  body_link->AddLinkForce(thrust_val*thrust_unit_vec, pose.Pos());

  if (_info.simTime - last_published_msg > publish_period) {
    // update our time
    last_published_msg = _info.simTime;

    // publish our wrench for visualization
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp.fromSec(_info.simTime.Float());
    msg.header.frame_id = thruster_link;
    msg.wrench.force.x = thrust_val;
    msg.wrench.force.y = 0;
    msg.wrench.force.z = 0;

    viz_pub.publish(msg);

    // if the vehicle's thruster driver isn't running, go ahead and published an ideal thruster state
    if (!vehicle_in_loop) {
      ds_actuator_msgs::ThrusterState state_msg;
      state_msg.header = msg.header;
      state_msg.ds_header.io_time = state_msg.header.stamp;
      state_msg.enable = enabled;
      state_msg.thruster_name = thruster_name;
      state_msg.cmd_value = command;
      state_msg.actual_value = command;

      state_pub.publish(state_msg);
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(DsrosThruster)