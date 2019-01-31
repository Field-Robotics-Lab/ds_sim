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

  body_link->SetGravityMode(false);

  // /////////////////////////////////////////////////////////////////////// //
  GZ_ASSERT(_sdf->HasElement("drag"), "DsrosHydro MUST specify a drag element");
  sdf::ElementPtr drag = _sdf->GetElement("drag");

  // this vector has to stay relative to the link origin, because that
  // function we use to apply the relative force is relative to the link origin.
  // Unlike ALL THE OTHER apply force functions.  Gazebo never met a coordinate system
  // it could get documented.
  if (!drag->HasElement("center")) {
    gzwarn <<"ds_hydro plugin for link " <<body_link->GetName()
           <<" does not specify a center of hydrodynamic force; assuming Center of Mass" <<std::endl;
    drag_center_body = body_link->GetInertial()->GetCoG();
  } else {
    drag_center_body = loadVector(drag->GetElement("center"));
  }

  if (drag->HasElement("linear")) {
    drag_lin_coeff = loadMatrix(drag->GetElement("linear"));
  } else {
    gzwarn <<"ds_hydro plugin for link " <<body_link->GetName()
           <<" does not specify linear drag matrix! (MAY CAUSE INSTABILITY!)" <<std::endl;
  }

  if (drag->HasElement("quadratic")) {
    drag_lin_coeff = loadMatrix(drag->GetElement("quadratic"));
  } else {
    gzwarn <<"ds_hydro plugin for link " <<body_link->GetName()
           <<" does not specify quadratic drag matrix!" <<std::endl;
  }

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
  Matrix6d ret;
  ret.m11 = math::Matrix3::ZERO;
  ret.m12 = math::Matrix3::ZERO;
  ret.m21 = math::Matrix3::ZERO;
  ret.m22 = math::Matrix3::ZERO;

  std::vector<std::string> axis_tags = {"x", "y", "z", "r", "p", "h"};

  for (size_t i=0; i<3; i++) {
    for (size_t j=0; j<3; j++) {
      // the top-left segment
      std::string tag11 = axis_tags[i]   + axis_tags[j];
      std::string tag12 = axis_tags[i]   + axis_tags[j+3];
      std::string tag21 = axis_tags[i+3] + axis_tags[j];
      std::string tag22 = axis_tags[i+3] + axis_tags[j+3];

      if (sdf->HasElement(tag11)) {
        ret.m11[i][j] = sdf->Get<double>(tag11);
      }
      if (sdf->HasElement(tag12)) {
        ret.m12[i][j] = sdf->Get<double>(tag12);
      }
      if (sdf->HasElement(tag21)) {
        ret.m21[i][j] = sdf->Get<double>(tag21);
      }
      if (sdf->HasElement(tag22)) {
        ret.m22[i][j] = sdf->Get<double>(tag22);
      }
    }
  }

  //gzmsg <<"M_11: " <<ret.m11 <<"\n";
  //gzmsg <<"M_12: " <<ret.m12 <<"\n";
  //gzmsg <<"M_21: " <<ret.m21 <<"\n";
  //gzmsg <<"M_22: " <<ret.m22 <<"\n";

  return ret;
}

void DsrosHydro::OnUpdate(const common::UpdateInfo& _info) {

  // Apply gravity
  body_link->AddForce(grav_force_world); // works at COM

  // Apply buoyancy.  Note that this is relative to the CENTER OF MASS, not the link
  // origin.  I share your dream of Gazebo someday documenting their coordinate frames
  body_link->AddForceAtRelativePosition(buoy_force_world, buoy_center_com);

  // Add linear drag
  math::Vector3 vel_lin = body_link->GetRelativeLinearVel();
  math::Vector3 vel_ang = body_link->GetRelativeAngularVel();

  math::Vector3 drag_lin_force = drag_lin_coeff.m11 * vel_lin + drag_lin_coeff.m12 * vel_ang;
  math::Vector3 drag_lin_torque = drag_lin_coeff.m21 * vel_lin + drag_lin_coeff.m22 * vel_ang;

  body_link->AddLinkForce(-drag_lin_force, drag_center_body);
  body_link->AddRelativeTorque(-drag_lin_torque);

  // Add quadratic drag
  math::Vector3 vel_lin2 = vel_lin*vel_lin;
  math::Vector3 vel_ang2 = vel_ang*vel_ang;

  math::Vector3 drag_quad_force = drag_quad_coeff.m11 * vel_lin2 + drag_quad_coeff.m12 * vel_ang2;
  math::Vector3 drag_quad_torque = drag_quad_coeff.m21 * vel_lin2 + drag_quad_coeff.m22 * vel_ang2;

  body_link->AddLinkForce(-drag_quad_force, drag_center_body);
  body_link->AddRelativeTorque(-drag_quad_torque);

  // Add a phantom "added mass" term
  // TODO

  // This KILLs performance
  // check total forces at the end
  //gzmsg <<"WORLD force: " <<body_link->GetWorldForce() <<" " <<body_link->GetWorldTorque() <<"\n";

}

