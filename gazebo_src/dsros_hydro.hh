//
// Created by ivaughn on 1/29/19.
//

#ifndef PROJECT_DSROS_HYDRO_HH
#define PROJECT_DSROS_HYDRO_HH

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Damn you vile Gazebo
struct Matrix6d {
  Matrix6d() {
    m11 = ignition::math::Matrix3<double>::Zero;
    m12 = ignition::math::Matrix3<double>::Zero;
    m21 = ignition::math::Matrix3<double>::Zero;
    m22 = ignition::math::Matrix3<double>::Zero;
  }

  ignition::math::Matrix3<double> m11;
  ignition::math::Matrix3<double> m12;
  ignition::math::Matrix3<double> m21;
  ignition::math::Matrix3<double> m22;
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

  ignition::math::Vector3<double> buoy_center_com; // center-of-mass relative
  ignition::math::Vector3<double> buoy_force_world;
  double buoy_all_buoyancy_depth; // depth at which we start losing bouyancy
  double buoy_no_buoyancy_depth; // depth at which the vehicle is fully out of the water
  double buoy_surface_scale; // scale factor to reduce oscillation

  ignition::math::Vector3<double> grav_center_body;
  ignition::math::Vector3<double> grav_force_world;
  ignition::math::Vector3<double> com_vector; // center of mass relative to link origin

  ignition::math::Vector3<double> drag_center_body; // center-of-mass relative
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
  ignition::math::Vector3<double> filt_acc_lin;
  ignition::math::Vector3<double> filt_acc_ang;
  ignition::math::Vector3<double> previous_relvel_lin;
  ignition::math::Vector3<double> previous_relvel_ang;
  common::Time previous_time;

  ignition::math::Vector3<double> loadVector(sdf::ElementPtr sdf);
  Matrix6d loadMatrix(sdf::ElementPtr sdf);
  std::array<Matrix6d, 6> loadTensor(sdf::ElementPtr sdf);
};
}
#endif //PROJECT_DSROS_HYDRO_HH
