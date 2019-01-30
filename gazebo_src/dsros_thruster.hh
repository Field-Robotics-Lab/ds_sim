//
// Created by ivaughn on 1/29/19.
//

#ifndef PROJECT_DSROS_THRUSTER_HH
#define PROJECT_DSROS_THRUSTER_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {

class DsrosThruster : public ModelPlugin {

 public:
  DsrosThruster();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& _info);

 protected:



};
}

#endif //PROJECT_DSROS_THRUSTER_HH
