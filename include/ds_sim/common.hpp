/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
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


#if GAZEBO_MAJOR_VERSION > 7
template <typename T>
void fillTwist(T& ret, const ignition::math::Vector3d& xyz, const ignition::math::Vector3d& rph) {
#else
template <typename T>
void fillTwist(T& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
#endif
    ret.linear.x = xyz[0];
    ret.linear.y = xyz[1];
    ret.linear.z = xyz[2];
    ret.angular.x= rph[0];
    ret.angular.y= rph[1];
    ret.angular.z= rph[2];
}

#if GAZEBO_MAJOR_VERSION > 7
void fillWrench(geometry_msgs::Wrench& ret, const ignition::math::Vector3d& xyz, const ignition::math::Vector3d& rph) {
#else
void fillWrench(geometry_msgs::Wrench& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
#endif
    ret.force.x = xyz[0];
    ret.force.y = xyz[1];
    ret.force.z = xyz[2];
    ret.torque.x= rph[0];
    ret.torque.y= rph[1];
    ret.torque.z= rph[2];
}

#if GAZEBO_MAJOR_VERSION > 7
template <typename T>
void fillTwistFossen(T& ret, const ignition::math::Vector3d& xyz, const ignition::math::Vector3d& rph) {
#else
template <typename T>
void fillTwistFossen(T& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
#endif
    ret.linear.x = xyz[0];
    ret.linear.y = -xyz[1];
    ret.linear.z = -xyz[2];
    ret.angular.x= rph[0];
    ret.angular.y= -rph[1];
    ret.angular.z= -rph[2];
}

#if GAZEBO_MAJOR_VERSION > 7
void fillWrenchFossen(geometry_msgs::Wrench& ret, const ignition::math::Vector3d& xyz, const ignition::math::Vector3d& rph) {
#else
void fillWrenchFossen(geometry_msgs::Wrench& ret, const gazebo::math::Vector3& xyz, const gazebo::math::Vector3& rph) {
#endif
    ret.force.x = xyz[0];
    ret.force.y = -xyz[1];
    ret.force.z = -xyz[2];
    ret.torque.x= rph[0];
    ret.torque.y= -rph[1];
    ret.torque.z= -rph[2];
}

 
/// \brief Convert a fossen pose to a gazebo pose
///
#if GAZEBO_MAJOR_VERSION > 7
void fossen2gazebo_pose(ignition::math::Pose3d& gz_pose, const geometry_msgs::Pose& fossen_pose) {
#else
void fossen2gazebo_pose(gazebo::math::Pose& gz_pose, const geometry_msgs::Pose& fossen_pose) {
#endif
  
  // TODO: Figure out how to do this math for quaternions directly
  // first, create a gazebo quaternion to do the conversion from quaternion to roll/pitch/heading
  #if GAZEBO_MAJOR_VERSION > 7
  ignition::math::Quaterniond tmp(fossen_pose.orientation.w, fossen_pose.orientation.x,
                               fossen_pose.orientation.y, fossen_pose.orientation.z);
  #else
  gazebo::math::Quaternion tmp(fossen_pose.orientation.w, fossen_pose.orientation.x,
                               fossen_pose.orientation.y, fossen_pose.orientation.z);
  #endif

  // Recover roll/pitch/heading
  #if GAZEBO_MAJOR_VERSION > 7
  double roll  = tmp.Roll();
  double pitch = tmp.Pitch();
  double yaw   = tmp.Yaw();
  #else
  double roll  = tmp.GetRoll();
  double pitch = tmp.GetPitch();
  double yaw   = tmp.GetYaw();
  #endif
  
  // do the conversion to gazebo
  #if GAZEBO_MAJOR_VERSION > 7
  gz_pose.Set(ignition::math::Vector3d(fossen_pose.position.y, fossen_pose.position.x, -fossen_pose.position.z), ignition::math::Quaterniond(roll, -pitch, M_PI/2.0-yaw));
  #else
  gz_pose.rot = gazebo::math::Quaternion(roll, -pitch, M_PI/2.0-yaw);
  // flip x/y, invert z
  gz_pose.pos.x = fossen_pose.position.y;
  gz_pose.pos.y = fossen_pose.position.x;
  gz_pose.pos.z = -fossen_pose.position.z;
  #endif

  
};

/// \brief Convert a gazebo pose to a fossen pose
///
/// \param fossen_pose The destination
/// \param gz_pose  The source (NOTE: SHOULD REALLY BE CONST, but gazebo developers don't have a const accessor??
#if GAZEBO_MAJOR_VERSION > 7
void gazebo2fossen_pose(geometry_msgs::Pose& fossen_pose, ignition::math::Pose3d& gz_pose) {
#else
void gazebo2fossen_pose(geometry_msgs::Pose& fossen_pose, gazebo::math::Pose& gz_pose) {
#endif

  // TODO: Figure out how to do this math for quaternions directly

  // first, create a gazebo quaternion to do the conversion from quaternion to roll/pitch/heading
  // create a temporary quaternion
  #if GAZEBO_MAJOR_VERSION > 7
  ignition::math::Quaterniond tmp(gz_pose.Rot().Roll(), -gz_pose.Rot().Pitch(), M_PI/2.0-gz_pose.Rot().Yaw());
  fossen_pose.orientation.x = tmp.X();
  fossen_pose.orientation.y = tmp.Y();
  fossen_pose.orientation.z = tmp.Z();
  fossen_pose.orientation.w = tmp.W();
  // flip x/y, invert z
  fossen_pose.position.x = gz_pose.Pos().Y();
  fossen_pose.position.y = gz_pose.Pos().X();
  fossen_pose.position.z = -gz_pose.Pos().Z();
  #else
  gazebo::math::Quaternion tmp(gz_pose.rot.GetRoll(), -gz_pose.rot.GetPitch(), M_PI/2.0-gz_pose.rot.GetYaw());
  fossen_pose.orientation.x = tmp.x;
  fossen_pose.orientation.y = tmp.y;
  fossen_pose.orientation.z = tmp.z;
  fossen_pose.orientation.w = tmp.w;
  // flip x/y, invert z
  fossen_pose.position.x = gz_pose.pos.y;
  fossen_pose.position.y = gz_pose.pos.x;
  fossen_pose.position.z = -gz_pose.pos.z;
  #endif
};

#if GAZEBO_MAJOR_VERSION > 7
void body2world_rates(ignition::math::Vector3d& gz_world_rates, ignition::math::Quaterniond& gz_att, ignition::math::Vector3d body_rates_gz) {
#else
void body2world_rates(gazebo::math::Vector3& gz_world_rates, gazebo::math::Quaternion& gz_att, gazebo::math::Vector3 body_rates_gz) {
#endif
  // even though we take our inputs in gazebo's ENU / FPU coordinate frame, we immediately convert
  // to fossen so we can use fossen's math
  // Body rates are roll, pitch, yaw
  #if GAZEBO_MAJOR_VERSION > 7
    double roll = gz_att.Roll();
    double pitch = -gz_att.Pitch();
    double yaw = M_PI/2 - gz_att.Yaw();
    double sr = sin(roll);
    double cr = cos(roll);
    double sp = sin(pitch);
    double cp = cos(pitch);
    double tp = tan(pitch);

    // convert the body_rates_gz on the fly to FSD from FPU (invert 1 & 2)
    // gz_world_rates will be in FOSSEN!-style notation
    gz_world_rates.X(body_rates_gz[0] - body_rates_gz[1]*sr*tp - body_rates_gz[2]*cr*tp);
    gz_world_rates.Y(-1.0 * -body_rates_gz[1] * cr + body_rates_gz[2] * sr);
    gz_world_rates.Z(-1.0 * -body_rates_gz[1] * sr/cp - body_rates_gz[2] * cr/cp);

    // Flip the signs to get back to gazebo's ENU body frame -- SS sign change embedded in the above math
    //gz_world_rates.y *= -1.0;
    //gz_world_rates.z *= -1.0;
  #else
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
  #endif
}
