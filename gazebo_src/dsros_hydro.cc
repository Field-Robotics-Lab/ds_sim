//
// Created by ivaughn on 1/29/19.
//


#include "dsros_hydro.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(DsrosHydro)

DsrosHydro::DsrosHydro() {
}

void DsrosHydro::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  // /////////////////////////////////////////////////////////////////////// //
  GZ_ASSERT(_sdf->HasElement("base_link"), "DsrosHydro MUST specify a base link");
  frame_name = _sdf->Get<std::string>("base_link");
  gzmsg <<"Adding hydrodynamics to link " <<_parent->GetLink()->GetName();
  body_link = _parent->GetLink();

  // /////////////////////////////////////////////////////////////////////// //
  GZ_ASSERT(_sdf->HasElement("buoyancy"), "DsrosHydro MUST specify a buoyancy element");
  sdf::ElementPtr buoy = _sdf->GetElement("buoyancy");

  // Get the center-of-buoyancy
  GZ_ASSERT(buoy->HasElement("center"), "Buoyancy tag MUST specify a center tag");
  buoy_center_com = loadVector(buoy->GetElement("center")) - body_link->GetInertial()->GetCoG();

  // Get the center-of-buoyancy
  grav_center_body = body_link->GetInertial()->GetCoG();

  gzmsg <<"   Buoy Center: " <<buoy_center_com <<std::endl;
  gzmsg <<"Center Of Mass: " <<grav_center_body <<std::endl;

  // Build the buoyancy vector
  GZ_ASSERT(buoy->HasElement("buoyancy"), "Buoyancy tag MUST specify a buoyancy tag");
  double buoyancy = buoy->Get<double>("buoyancy");
  buoy_force_world = -buoyancy * (_parent->GetWorld()->Gravity());

  // Build the gravity vector
  GZ_ASSERT(buoy->HasElement("mass"), "Buoyancy tag MUST specify a mass tag");
  double mass = buoy->Get<double>("mass");
  grav_force_world = mass * (_parent->GetWorld()->Gravity());

  // /////////////////////////////////////////////////////////////////////// //
  GZ_ASSERT(_sdf->HasElement("buoyancy"), "DsrosHydro MUST specify a drag element");
  sdf::ElementPtr drag = _sdf->GetElement("drag");


  body_link->SetGravityMode(false);

  // /////////////////////////////////////////////////////////////////////// //
  // Connect the update handler
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DsrosHydro::OnUpdate, this, _1));

}

math::Vector3 DsrosHydro::loadVector(sdf::ElementPtr sdf) {
  math::Vector3 ret;

  //GZ_ASSERT(sdf->HasAttribute("x"), "Vector MUST specify \"x\"");
  ret.x = sdf->Get<double>("x");

  //GZ_ASSERT(sdf->HasElement("y"), "Vector MUST specify \"Y\"");
  ret.y = sdf->Get<double>("y");

  //GZ_ASSERT(sdf->HasElement("z"), "Vector MUST specify \"Z\"");
  ret.z = sdf->Get<double>("z");

  return ret;
}

Matrix6d DsrosHydro::loadMatrix(sdf::ElementPtr sdf) {

}

void DsrosHydro::OnUpdate(const common::UpdateInfo& _info) {

  // Apply gravity
  body_link->AddForce(grav_force_world); // works at COM

  // Apply buoyancy.  Note that this is relative to the CENTER OF MASS, not the link
  // origin.  I share your dream of Gazebo someday documenting their coordinate frames
  body_link->AddForceAtRelativePosition(buoy_force_world, buoy_center_com);

  // Add linear drag
  // TODO

  // Add quadratic drag
  // TODO

  // Add a phantom "added mass" term
  // TODO

  // Check total force at the end
  math::Vector3 total_force = body_link->GetWorldForce();
  math::Vector3 total_torque = body_link->GetWorldTorque();
  gzmsg <<"WORLD force: " <<total_force <<" " <<total_torque <<"\n";

}

