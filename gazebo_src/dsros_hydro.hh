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
  double buoy_min_depth;

  math::Vector3 grav_center_body;
  math::Vector3 grav_force_world;

  math::Vector3 drag_center_body; // center-of-mass relative
  Matrix6d drag_lin_coeff;
  Matrix6d drag_quad_coeff;

  math::Vector3 loadVector(sdf::ElementPtr sdf);
  Matrix6d loadMatrix(sdf::ElementPtr sdf);
};
}
#endif //PROJECT_DSROS_HYDRO_HH
