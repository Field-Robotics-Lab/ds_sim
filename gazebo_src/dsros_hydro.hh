//
// Created by ivaughn on 1/29/19.
//

#ifndef PROJECT_DSROS_HYDRO_HH
#define PROJECT_DSROS_HYDRO_HH

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>

// Damn you vile Gazebo
struct Matrix6d {
  Matrix6d() {
    m11 = gazebo::math::Matrix3::ZERO;
    m12 = gazebo::math::Matrix3::ZERO;
    m21 = gazebo::math::Matrix3::ZERO;
    m22 = gazebo::math::Matrix3::ZERO;
  }

  gazebo::math::Matrix3 m11;
  gazebo::math::Matrix3 m12;
  gazebo::math::Matrix3 m21;
  gazebo::math::Matrix3 m22;
};

namespace gazebo {

class DsrosHydro : public ModelPlugin {

 public:
  DsrosHydro();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& _info);

 protected:
  std::string frame_name;
  gazebo::physics::LinkPtr body_link;
  gazebo::event::ConnectionPtr updateConnection;

  math::Vector3 buoy_center_com; // center-of-mass relative
  math::Vector3 buoy_force_world;
  double buoy_all_buoyancy_depth; // depth at which we start losing bouyancy
  double buoy_no_buoyancy_depth; // depth at which the vehicle is fully out of the water
  double buoy_surface_scale; // scale factor to reduce oscillation

  math::Vector3 grav_center_body;
  math::Vector3 grav_force_world;

  math::Vector3 drag_center_body; // center-of-mass relative
  Matrix6d drag_lin_coeff;

  // quadratic drag is a 6 x 6 x 6 tensor; see the modelling thing.
  std::array<Matrix6d, 6> drag_quad_coeff;

  // Added mass coefficients.  Coriolis and Centripital matrices will be built on the fly
  Matrix6d drag_added_mass_coeff;
  // added mass has this known issue where because accelerations instantaneously increase,
  // (because math sim), the acceleration at a given time step can cause thrashing when adding mass.
  // The solution is to force acceleration to slowly ramp up.
  // We ALSO have to compute our own relative accelerations, because gazebo ALSO screws that up.
  double acc_alpha; // decay coefficient for filtering
  math::Vector3 filt_acc_lin;
  math::Vector3 filt_acc_ang;
  math::Vector3 previous_relvel_lin;
  math::Vector3 previous_relvel_ang;
  common::Time previous_time;

  math::Vector3 loadVector(sdf::ElementPtr sdf);
  Matrix6d loadMatrix(sdf::ElementPtr sdf);
  std::array<Matrix6d, 6> loadTensor(sdf::ElementPtr sdf);
};
}
#endif //PROJECT_DSROS_HYDRO_HH
