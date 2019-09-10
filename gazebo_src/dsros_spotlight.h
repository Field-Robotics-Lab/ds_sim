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


/// This is a hack to put headlights on a vehicle.  It has a lot to do with
/// old versions of gazebo, the ongoing debacle that is SDF vs. URDF, etc.
/// Basically, you have to define a light in your WORLD file,
/// then can use this plugin to attach it to the vehicle.
/// This plugin will update hte light's position based on the vehicle's position
/// at the end of every timestep, and it should look OK.
///
/// Unfortunatley, Gazebo insists on drawing a frustum showing the light in the
/// the simulation.  My attempts to turn that off so far ALSO turn off the light,
/// so for now we're giving up and moving on.
class dsrosSpotlight : public ModelPlugin {
 public:
  dsrosSpotlight();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& _info);

  void updateLightPosition();

 protected:
  gazebo::physics::LinkPtr body_link;
  ignition::math::Pose3d pose;

  //gazebo::rendering::LightPtr light;
  gazebo::physics::LightPtr light;

  std::string light_name;
  gazebo::event::ConnectionPtr updateConnection;
  gazebo::transport::NodePtr node;
  gazebo::transport::PublisherPtr requestPub;

};
}

#endif //DS_SIM_DSROS_SPOTLIGHT_H
