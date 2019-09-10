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

  light_name = _sdf->Get<std::string>("light_name");

  // Read the light's body-frame pose
  pose = _sdf->Get<ignition::math::Pose3d>("pose");

  bool hide_in_client = false;
  if (_sdf->HasElement("hide_in_client")) {
    hide_in_client = _sdf->Get<bool>("hide_in_client");
  }

  // prepare our publisher
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->requestPub = this->node->Advertise<msgs::Light>("~/light/modify");

  if (hide_in_client) {
    // turn off the visual
    gazebo::transport::PublisherPtr visual_pub = this->node->Advertise<msgs::Visual>("~/visual");
    msgs::Visual visual_msg;
    visual_msg.set_name(light_name);
    visual_msg.set_parent_name("");
    visual_msg.set_visible(false);

    visual_pub->Publish(visual_msg);
  }

  updateLightPosition();

  // wire some updates
  this->updateConnection = event::Events::ConnectWorldUpdateEnd(
      std::bind(&dsrosSpotlight::updateLightPosition, this));

}

void dsrosSpotlight::updateLightPosition() {
  ignition::math::Pose3d body_pose = body_link->GetWorldPose().Ign();
  ignition::math::Pose3d final_pose = pose + body_pose;
  //light->SetPosition(final_pose.Pos());
  //light->SetRotation(final_pose.Rot());

  if (light) {
    light->SetWorldPose(final_pose);
  }
  /*
  gzdbg <<"\n\n";
  gzdbg <<"World Pose: " <<body_pose <<"\n";
  gzdbg <<"      Pose: " <<pose <<"\n";
  gzdbg <<"Final Pose: " <<final_pose <<"\n\n" <<std::endl;
  */

  msgs::Light msg;
  msg.set_name(light_name);
  msgs::Set(msg.mutable_pose(), final_pose);

  requestPub->Publish(msg);

}
