#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>

template <typename T>
bool loadElement(T& result, const sdf::ElementPtr& _sdf, const std::string& tag) {
    if (!_sdf->HasElement(tag)) {
        ROS_FATAL_STREAM("ROS plugin sentrydynamics tag \""<<_sdf->GetName() 
                         <<"\" requires an attribute \"" <<tag <<"\"");
        return false;
    }

    result = _sdf->Get<T>(tag);

    ROS_INFO_STREAM("ROS plugin sentrydynamics/servo load \"" <<tag <<"\" = \"" <<result <<"\"");
    return true;
}

// specialization for string
template <>
bool loadElement<std::string>(std::string& result, const sdf::ElementPtr& _sdf, const std::string& tag) {
    if (!_sdf->HasElement(tag)) {
        ROS_FATAL_STREAM("ROS plugin sentrydynamics tag \""<<_sdf->GetName() 
                         <<"\" requires an attribute \"" <<tag <<"\"");
        return false;
    }

    result = _sdf->Get<std::string>(tag);
    // remove any double-quotes
    result.erase(std::remove(result.begin(), result.end(), '\"'),result.end());

    ROS_INFO_STREAM("ROS plugin sentrydynamics/servo load \"" <<tag <<"\" = \"" <<result <<"\"");
    return true;
}

template <typename T>
bool loadAttribute(T& result, const sdf::ElementPtr& _sdf, const std::string& tag) {
    if (!_sdf->HasAttribute(tag)) {
        ROS_FATAL_STREAM("ROS plugin sentrydynamics tag \""<<_sdf->GetName() 
                         <<"\" requires an attribute \"" <<tag <<"\"");
        return false;
    }
    sdf::ParamPtr param = _sdf->GetAttribute(tag);
    if (!param) {
        ROS_FATAL_STREAM("ROS plugin sentrydynamics tag \""<<_sdf->GetName() 
                         <<"\" failed to parse attribute \"" <<tag <<"\"");
        return false;
    }

    return param->Get(result);
}

// specialization for string
template <>
bool loadAttribute<std::string>(std::string& result, const sdf::ElementPtr& _sdf, const std::string& tag) {
    if (!_sdf->HasAttribute(tag)) {
        ROS_FATAL_STREAM("ROS plugin sentrydynamics tag \""<<_sdf->GetName() 
                         <<"\" requires an attribute \"" <<tag <<"\"");
        return false;
    }
    sdf::ParamPtr param = _sdf->GetAttribute(tag);
    if (!param) {
        ROS_FATAL_STREAM("ROS plugin sentrydynamics tag \""<<_sdf->GetName() 
                         <<"\" failed to parse attribute \"" <<tag <<"\"");
        return false;
    }

    bool ret = param->Get(result);
    if (ret) {
        // remove any double-quotes
        result.erase(std::remove(result.begin(), result.end(), '\"'),result.end());
    }
    return ret;
}

template <typename T>
void fillTwist(T& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
    ret.linear.x = xyz[0];
    ret.linear.y = xyz[1];
    ret.linear.z = xyz[2];
    ret.angular.x= rph[0];
    ret.angular.y= rph[1];
    ret.angular.z= rph[2];
}
void fillWrench(geometry_msgs::Wrench& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
    ret.force.x = xyz[0];
    ret.force.y = xyz[1];
    ret.force.z = xyz[2];
    ret.torque.x= rph[0];
    ret.torque.y= rph[1];
    ret.torque.z= rph[2];
}


template <typename T>
void fillTwistFossen(T& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
    ret.linear.x = xyz[0];
    ret.linear.y = -xyz[1];
    ret.linear.z = -xyz[2];
    ret.angular.x= rph[0];
    ret.angular.y= -rph[1];
    ret.angular.z= -rph[2];
}
void fillWrenchFossen(geometry_msgs::Wrench& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
    ret.force.x = xyz[0];
    ret.force.y = -xyz[1];
    ret.force.z = -xyz[2];
    ret.torque.x= rph[0];
    ret.torque.y= -rph[1];
    ret.torque.z= -rph[2];
}

/// \brief Convert a fossen pose to a gazebo pose
///
void fossen2gazebo_pose(gazebo::math::Pose& gz_pose, const geometry_msgs::Pose& fossen_pose) {

  // TODO: Figure out how to do this math for quaternions directly
  // first, create a gazebo quaternion to do the conversion from quaternion to roll/pitch/heading
  gazebo::math::Quaternion tmp(fossen_pose.orientation.w, fossen_pose.orientation.x,
                               fossen_pose.orientation.y, fossen_pose.orientation.z);

  // Recover roll/pitch/heading
  double roll  = tmp.GetRoll();
  double pitch = tmp.GetPitch();
  double yaw   = tmp.GetYaw();

  // do the conversion to gazebo
  gz_pose.rot = gazebo::math::Quaternion(roll, -pitch, M_PI/2.0-yaw);

  // flip x/y, invert z
  gz_pose.pos.x = fossen_pose.position.y;
  gz_pose.pos.y = fossen_pose.position.x;
  gz_pose.pos.z = -fossen_pose.position.z;
};

/// \brief Convert a gazebo pose to a fossen pose
///
/// \param fossen_pose The destination
/// \param gz_pose  The source (NOTE: SHOULD REALLY BE CONST, but gazebo developers don't have a const accessor??
void gazebo2fossen_pose(geometry_msgs::Pose& fossen_pose, gazebo::math::Pose& gz_pose) {

  // TODO: Figure out how to do this math for quaternions directly

  // first, create a gazebo quaternion to do the conversion from quaternion to roll/pitch/heading
  // create a temporary quaternion
  gazebo::math::Quaternion tmp(gz_pose.rot.GetRoll(), -gz_pose.rot.GetPitch(), M_PI/2.0-gz_pose.rot.GetYaw());
  fossen_pose.orientation.x = tmp.x;
  fossen_pose.orientation.y = tmp.y;
  fossen_pose.orientation.z = tmp.z;
  fossen_pose.orientation.w = tmp.w;

  // flip x/y, invert z
  fossen_pose.position.x = gz_pose.pos.y;
  fossen_pose.position.y = gz_pose.pos.x;
  fossen_pose.position.z = -gz_pose.pos.z;
};

void body2world_rates(gazebo::math::Vector3& gz_world_rates, gazebo::math::Quaternion& gz_att, gazebo::math::Vector3 body_rates_gz) {
  // even though we take our inputs in gazebo's ENU / FPU coordinate frame, we immediately convert
  // to fossen so we can use fossen's math
  // Body rates are roll, pitch, yaw
  double roll = gz_att.GetRoll();
  double pitch = -gz_att.GetPitch();
  double yaw = M_PI/2 - gz_att.GetYaw();
  double sr = sin(roll);
  double cr = cos(roll);
  double sp = sin(pitch);
  double cp = cos(pitch);
  double tp = tan(pitch);

  // convert the body_rates_gz on the fly to FSD from FPU (invert 1 & 2)
  // gz_world_rates will be in FOSSEN!-style notation
  gz_world_rates.x = body_rates_gz[0] - body_rates_gz[1]*sr*tp - body_rates_gz[2]*cr*tp;
  gz_world_rates.y = -body_rates_gz[1] * cr + body_rates_gz[2] * sr;
  gz_world_rates.z = -body_rates_gz[1] * sr/cp - body_rates_gz[2] * cr/cp;

  // Flip the signs to get back to gazebo's ENU body frame
  gz_world_rates.y *= -1.0;
  gz_world_rates.z *= -1.0;
}