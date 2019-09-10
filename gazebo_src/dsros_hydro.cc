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

  if (buoy->HasElement("scale_factor")) {
    buoy_surface_scale = buoy->Get<double>("scale_factor");
  } else {
    buoy_surface_scale = 1.0;
  }
  if (buoy->HasElement("out_of_water_depth")) {
    buoy_no_buoyancy_depth = buoy->Get<double>("out_of_water_depth");
  } else {
    buoy_no_buoyancy_depth = 0;
  }

  if (buoy->HasElement("fully_submerged_depth")) {
    buoy_all_buoyancy_depth = buoy->Get<double>("fully_submerged_depth");
  } else {
    buoy_all_buoyancy_depth = 0;
  }

  // Get the center-of-buoyancy
  grav_center_body = body_link->GetInertial()->GetCoG();

  gzmsg <<"   Buoy Center: " <<buoy_center_com <<std::endl;
  gzmsg <<"Center Of Mass: " <<grav_center_body <<std::endl;
  gzmsg <<"Buoyancy scaling active betweeen: " <<buoy_no_buoyancy_depth <<" and " <<buoy_all_buoyancy_depth <<"\n";

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
  //gzmsg <<" Loading center-of-drag...\n";
  if (!drag->HasElement("center")) {
    gzwarn <<"ds_hydro plugin for link " <<body_link->GetName()
           <<" does not specify a center of hydrodynamic force; assuming Center of Mass" <<std::endl;
    drag_center_body = body_link->GetInertial()->GetCoG();
  } else {
    drag_center_body = loadVector(drag->GetElement("center"));
  }

  //gzmsg <<" Loading linear drag...\n";
  if (drag->HasElement("linear")) {
    drag_lin_coeff = loadMatrix(drag->GetElement("linear"));
  } else {
    gzwarn <<"ds_hydro plugin for link " <<body_link->GetName()
           <<" does not specify linear drag matrix! (MAY CAUSE INSTABILITY!)" <<std::endl;
  }

  //gzmsg <<"Loading quadratic drag...\n";
  if (drag->HasElement("quadratic")) {
    drag_quad_coeff = loadTensor(drag->GetElement("quadratic"));
  } else {
    gzwarn <<"ds_hydro plugin for link " <<body_link->GetName()
           <<" does not specify quadratic drag tensor!" <<std::endl;
  }

  //gzmsg <<"Loading added_mass drag...\n";
  if (drag->HasElement("added_mass")) {
    drag_added_mass_coeff = loadMatrix(drag->GetElement("added_mass"));
  } else {
    gzwarn <<"ds_hydro plugin for link " <<body_link->GetName()
           <<" does not specify added mass matrix!" <<std::endl;
  }

  // initialize some stuff
  acc_alpha = 0.3;
  filt_acc_lin = math::Vector3::Zero;
  filt_acc_ang = math::Vector3::Zero;
  previous_relvel_lin = math::Vector3::Zero;
  previous_relvel_ang = math::Vector3::Zero;

  // /////////////////////////////////////////////////////////////////////// //
  // Connect the update handler
  //gzmsg <<"Updating connection handler for dsros_hydro plugin!\n";
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

std::array<Matrix6d, 6> DsrosHydro::loadTensor(sdf::ElementPtr sdf) {
  std::array<Matrix6d, 6> ret;
  for (size_t i=0; i<6; i++) {
    ret[i].m11 = math::Matrix3::ZERO;
    ret[i].m12 = math::Matrix3::ZERO;
    ret[i].m21 = math::Matrix3::ZERO;
    ret[i].m22 = math::Matrix3::ZERO;
  }

  std::vector<std::string> axis_tags = {"x", "y", "z", "r", "p", "h"};

  for (size_t i=0; i<6; i++) { // j,k read BOTH linear & angular components; i must go through all 6
    for (size_t j = 0; j < 3; j++) {
      for (size_t k = 0; k < 3; k++) {

        // Ok, so the individual matrices are storing j,k
        // Each matrix is a slice by i, so the at we can write to a single
        // output axis as a matrix quadratic product

        // the top-left segment
        std::string tag11 = axis_tags[i] + axis_tags[j] + axis_tags[k];
        std::string tag12 = axis_tags[i] + axis_tags[j] + axis_tags[k + 3];
        std::string tag21 = axis_tags[i] + axis_tags[j + 3] + axis_tags[k];
        std::string tag22 = axis_tags[i] + axis_tags[j + 3] + axis_tags[k + 3];

        if (sdf->HasElement(tag11)) {
          ret[i].m11[j][k] = sdf->Get<double>(tag11);
        }
        if (sdf->HasElement(tag12)) {
          ret[i].m12[j][k] = sdf->Get<double>(tag12);
        }
        if (sdf->HasElement(tag21)) {
          ret[i].m21[j][k] = sdf->Get<double>(tag21);
        }
        if (sdf->HasElement(tag22)) {
          ret[i].m22[j][k] = sdf->Get<double>(tag22);
        }
      }
    }
  }

  return ret;
}

gazebo::math::Matrix3 skew_symmetric(const math::Vector3& v) {
  gazebo::math::Matrix3 ret;
  // row 0
  ret[0][0] = 0;
  ret[0][1] = -v[2];
  ret[0][2] = v[1];

  // row 1
  ret[1][0] = v[2];
  ret[1][1] = 0;
  ret[1][2] = -v[0];

  // row 2
  ret[2][0] = -v[1];
  ret[2][1] = v[0];
  ret[2][2] = 0;

  return ret;
}

void DsrosHydro::OnUpdate(const common::UpdateInfo& _info) {

  // compute timestep size, in seconds
  auto timestep = _info.simTime - previous_time;
  previous_time = _info.simTime;
  double dt = timestep.Double();
  if (dt > 1.0) {
    gzwarn <<"Timestep for Hydro Sim TOO BIG!, skipping update...\n";
    return;
  }

  // Apply gravity
  body_link->AddForce(grav_force_world); // works at COM

  // Apply buoyancy.  Note that this is relative to the CENTER OF MASS, not the link
  // origin.  I share your dream of Gazebo someday documenting their coordinate frames
  double depth = -(body_link->GetWorldPose().pos.z);

  if (depth >= buoy_all_buoyancy_depth) {
    // we're fully submerged
    body_link->AddForceAtRelativePosition(buoy_force_world, buoy_center_com);
  } else if (depth <= buoy_no_buoyancy_depth) {
    // we're fully out of the water, just let gravity do its thing.
    // Even though its in free-fall in air, we'll still apply drag as if in water.
    body_link->AddForceAtRelativePosition(math::Vector3::Zero, buoy_center_com);
  } else {
    // we're partially submerged; scale buoyancy linearly
    double dynamic_buoyancy = (depth - buoy_no_buoyancy_depth)/(buoy_all_buoyancy_depth - buoy_no_buoyancy_depth);
    // we only allow the scale_factor fraction fo bouyancy to change during surface interactions
    double buoy_scale_factor = buoy_surface_scale * dynamic_buoyancy + (1.0-buoy_surface_scale);
    body_link->AddForceAtRelativePosition(buoy_scale_factor*buoy_force_world, buoy_center_com);
  }

  // Add linear drag
  math::Vector3 vel_lin = body_link->GetRelativeLinearVel();
  math::Vector3 vel_ang = body_link->GetRelativeAngularVel();

  math::Vector3 drag_lin_force = drag_lin_coeff.m11 * vel_lin + drag_lin_coeff.m12 * vel_ang;
  math::Vector3 drag_lin_torque = drag_lin_coeff.m21 * vel_lin + drag_lin_coeff.m22 * vel_ang;

  body_link->AddLinkForce(-drag_lin_force, drag_center_body);
  body_link->AddRelativeTorque(-drag_lin_torque);

  // quadratic drag is much harder
  math::Vector3 absvel_lin = vel_lin.GetAbs();
  math::Vector3 absvel_ang = vel_ang.GetAbs();

  // this matrix library is officially the WORST
  std::array<double, 3> tmp_quad_force;
  std::array<double, 3> tmp_quad_torque;
  for (size_t i=0; i<3; i++) {

    // ok, so we WANT to compute |v|^T * C * v
    // But there's no way to do matrix products from the left in either gazebo math or
    // ignition, and they're not using Eigen because we just have to have a broken linear algebra library.
    // Instead do |v| dot (C*v).  Which is at least the same math.

    // its even messier because we split linear and angular values using hte whole m11-m22 nonsense.

    // linear components: i=0-2
    tmp_quad_force[i] = absvel_lin.Dot(drag_quad_coeff[i].m11*vel_lin)
        + absvel_ang.Dot(drag_quad_coeff[i].m21 * vel_lin)
        + absvel_lin.Dot(drag_quad_coeff[i].m12 * vel_ang)
        + absvel_ang.Dot(drag_quad_coeff[i].m22 * vel_ang);

    // angular components: i=3-5
    tmp_quad_torque[i] = absvel_lin.Dot(drag_quad_coeff[i+3].m11*vel_lin)
        + absvel_ang.Dot(drag_quad_coeff[i+3].m21 * vel_lin)
        + absvel_lin.Dot(drag_quad_coeff[i+3].m12 * vel_ang)
        + absvel_ang.Dot(drag_quad_coeff[i+3].m22 * vel_ang);
  }
  math::Vector3 drag_quad_force(tmp_quad_force[0], tmp_quad_force[1], tmp_quad_force[2]);
  math::Vector3 drag_quad_torque(tmp_quad_torque[0], tmp_quad_torque[1], tmp_quad_torque[2]);

  body_link->AddLinkForce(-drag_quad_force, drag_center_body);
  body_link->AddRelativeTorque(-drag_quad_torque);

  // Add a phantom "added mass" term to represent various bits of hydrodynamic forcing.
  // Added mass itself is like an acceleration drag; essentially just
  // F = -m_a * a

  // however, we first have to compute acceleration.  Which Gazebo makes stupidly hard. First, the accelerations
  // are way wrong.  Second, the only available acceleration grows instantaneously.  There's a note that the
  // right way to do this is to compute first the acceleration the body is expected to undergo this timestep,

  math::Vector3 acc_lin = (vel_lin - previous_relvel_lin)/dt;
  math::Vector3 acc_ang = (vel_ang - previous_relvel_ang)/dt;
  previous_relvel_lin = vel_lin;
  previous_relvel_ang = vel_ang;
  filt_acc_lin = (1.0 - acc_alpha) * filt_acc_lin + acc_alpha * acc_lin;
  filt_acc_ang = (1.0 - acc_alpha) * filt_acc_ang + acc_alpha * acc_ang;

  math::Vector3 added_mass_inertia_force = drag_added_mass_coeff.m11 * filt_acc_lin
      + drag_added_mass_coeff.m12 * filt_acc_ang;
  math::Vector3 added_mass_inertia_torque = drag_added_mass_coeff.m21 * filt_acc_lin
      + drag_added_mass_coeff.m22 * filt_acc_ang;

  // Added mass also contributes a coriolis and centrepital term, as
  // shown on Page 120 of Fossen's Handbook fo Marine Craft Hydrodynamics and Motion Control

  // compute the coriolois and centripetal matrix, Cam(v)
  Matrix6d added_mass_cam;
  math::Vector3 cam_lin = drag_added_mass_coeff.m11 * vel_lin + drag_added_mass_coeff.m12 * vel_ang;
  math::Vector3 cam_ang = drag_added_mass_coeff.m21 * vel_lin + drag_added_mass_coeff.m22 * vel_ang;

  // Fossen has a negative sign on all the skew-symmetric sub-matrices in (6.43) on page 120, but
  // it disappears in all the other equations-- notably (6.54). We include the negative sign essentially
  // because uuvsim does.
  added_mass_cam.m11 = math::Matrix3::ZERO;
  added_mass_cam.m12 = -1.0*skew_symmetric(cam_lin);
  added_mass_cam.m21 = added_mass_cam.m12;
  added_mass_cam.m22 = -1.0*skew_symmetric(cam_ang);

  math::Vector3 added_mass_cor_force = added_mass_cam.m11 * vel_lin + added_mass_cam.m12 * vel_ang;
  math::Vector3 added_mass_cor_torque = added_mass_cam.m21 * vel_lin + added_mass_cam.m22 * vel_ang;

  // add the inertial and coriolis/centripital term to get the final added mass contribution
  math::Vector3 added_mass_force = added_mass_inertia_force + added_mass_cor_force;
  math::Vector3 added_mass_torque = added_mass_inertia_torque + added_mass_cor_torque;

  // Forces are in the body frame, but applied to the CoM
  body_link->AddRelativeForce(-added_mass_force);
  body_link->AddRelativeTorque(-added_mass_torque);

  // This KILLs performance
  // check total forces at the end
  //gzmsg <<"WORLD force: " <<body_link->GetWorldForce() <<" " <<body_link->GetWorldTorque() <<"\n";

}

