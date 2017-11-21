#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
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


