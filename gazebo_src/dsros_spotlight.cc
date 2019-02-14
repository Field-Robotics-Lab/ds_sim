//
// Created by ivaughn on 2/14/19.
//

#include "dsros_spotlight.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(dsrosSpotlight)

dsrosSpotlight::dsrosSpotlight() {
}

void dsrosSpotlight::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  body_link = _parent->GetLink();

  // Create a light
  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzwarn <<"No scene, so not adding light!" <<std::endl;
    return;
  }
  light.reset(new rendering::Light(scene));
  // Read the light's config from the SDF
  light->Load(_sdf->GetElement("light"));

  // add the light to the world
  //scene->AddLight(light);

  // Read the light's body-frame pose
  pose = _sdf->Get<ignition::math::Pose3d>("pose");

  // set an initial position
  updateLightPosition();

  // wire some updates
  this->updateConnection = event::Events::ConnectWorldUpdateEnd(
      std::bind(&dsrosSpotlight::updateLightPosition, this));

}

void dsrosSpotlight::updateLightPosition() {
  ignition::math::Pose3d body_pose = body_link->GetWorldPose().Ign();
  ignition::math::Pose3d final_pose = pose + body_pose;
  light->SetPosition(final_pose.Pos());
  light->SetRotation(final_pose.Rot());
}
