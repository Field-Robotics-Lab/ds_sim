//
// Created by ivaughn on 1/29/19.
//

#ifndef PROJECT_DSROS_THRUSTER_HH
#define PROJECT_DSROS_THRUSTER_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ds_actuator_msgs/ThrusterCmd.h>

struct DsrosThrusterModel {
  DsrosThrusterModel() {
    gain = 0;
    offset = 0;
    speed_gain = 0;
    speed_offset = 0;
  }

  inline double calc_thrust(double cmd, double speed) {
    if (cmd > max_cmd) {
      cmd = max_cmd;
    }
    double value = gain*cmd + offset
        + speed_gain*cmd*speed + speed_offset*speed;
    if (value <= 0) {
      return 0;
    }
    return value;
  }

  double max_cmd;
  double gain;
  double offset;
  double speed_gain;
  double speed_offset;
};

namespace gazebo {

class DsrosThruster : public ModelPlugin {

 public:
  DsrosThruster();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& _info);
  void OnCmdUpdate(const ds_actuator_msgs::ThrusterCmd& cmd);

 protected:
  ignition::math::Pose3d pose;
  ignition::math::Vector3d thrust_unit_vec;
  std::string thruster_link;

  gazebo::physics::LinkPtr body_link;
  gazebo::event::ConnectionPtr updateConnection;

  common::Time command_timeout;
  double command;
  DsrosThrusterModel forward;
  DsrosThrusterModel reverse;

  std::unique_ptr<ros::NodeHandle> node_handle;
  ros::Publisher viz_pub;
  ros::Subscriber cmd_sub;



  DsrosThrusterModel LoadModel(sdf::ElementPtr sdf);




};
}

#endif //PROJECT_DSROS_THRUSTER_HH
