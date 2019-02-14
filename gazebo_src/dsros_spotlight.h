//
// Created by ivaughn on 2/14/19.
//

#ifndef DS_SIM_DSROS_SPOTLIGHT_H
#define DS_SIM_DSROS_SPOTLIGHT_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo {

class dsrosSpotlight : public ModelPlugin {
 public:
  dsrosSpotlight();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& _info);

  void updateLightPosition();

 protected:
  gazebo::physics::LinkPtr body_link;
  ignition::math::Pose3d pose;

  gazebo::rendering::LightPtr light;

  gazebo::event::ConnectionPtr updateConnection;
};
}

#endif //DS_SIM_DSROS_SPOTLIGHT_H
