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
/*
 *
 * dsros_inssensor.cc
 *
 * Ian Vaughn, 2017 Nov 16
 * 
 * A INS sensor.  This is NOT a plugin, its a whole
 * new sensor type.
 *
 * Loading it requires the epic dsros_sensors SystemPlugin hack.
 */

#include "dsros_ins.hh"

#include <boost/algorithm/string.hpp>

using namespace gazebo;
using namespace sensors;

DsrosInsSensor::DsrosInsSensor() : Sensor(sensors::OTHER){
    // do nothing
};

DsrosInsSensor::~DsrosInsSensor() {
    // do nothing
}

void DsrosInsSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf) {
    Sensor::Load(_worldName, _sdf);
} 

void DsrosInsSensor::Load(const std::string &_worldName) {
    Sensor::Load(_worldName);

    // Load the parent link
    physics::EntityPtr parentEntity = this->world->GetEntity(
                                                this->ParentName());
    this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);
    if (! this->parentLink) {
        gzerr <<"Sensor " <<this->Name() <<" could not find parent link!" <<std::endl;
        if (parentEntity) {
            gzerr <<"parent: " <<parentEntity->GetName() <<std::endl;
        }
    }

    // setup to publish!
    this->topicName = this->GetTopic();
    this->insPub = this->node->Advertise<ds_sim::msgs::Ins>(this->topicName, 50);
}

void DsrosInsSensor::Init() {
    Sensor::Init();
    this->sphericalCoordinates = this->world->GetSphericalCoordinates();
}

void DsrosInsSensor::Fini() {
    Sensor::Fini();
    parentLink.reset();
}

std::string DsrosInsSensor::GetTopic() const {
    std::string topicName = "~/" + this->ParentName() + '/' + this->Name();

    if (this->sdf->HasElement("topic")) {
        topicName += '/' + this->sdf->Get<std::string>("topic");
    } else {
        topicName += "/ins";
    }
    boost::replace_all(topicName, "::", "/");

    return topicName;
}

bool DsrosInsSensor::UpdateImpl(const bool _force) {
    std::lock_guard<std::mutex> lock(this->mutex);

    if (!this->parentLink) {
        return true;
    }

    // Get the actual sensor values

   
    ignition::math::Pose3d parentLinkPose = this->parentLink->GetWorldPose().Ign();
    ignition::math::Pose3d insPose = this->pose + parentLinkPose;

    // angular velocity
    // TODO This doesn't do what we want it to
    ignition::math::Vector3<double> angular_velocity
         = insPose.Rot().Inverse().RotateVector(this->parentLink->GetWorldAngularVel().Ign());

    // linear velocity
    ignition::math::Vector3d linear_velocity = this->parentLink->
                                    GetWorldLinearVel(this->pose.Pos(), this->pose.Rot()).Ign();

    // linear acceleration
    ignition::math::Vector3d gravity = this->world->GetPhysicsEngine()->GetGravity().Ign();
    ignition::math::Vector3d linear_accel = this->parentLink->GetWorldLinearAccel().Ign();
    ignition::math::Vector3d angular_accel = this->parentLink->GetWorldAngularAccel().Ign();
    // lever-arm offset
    linear_accel += angular_accel.Cross(this->pose.Pos());
    // TODO: There should probably be a centripital acceleration term or something?

    // apply gravity
    linear_accel -= insPose.Rot().Inverse().RotateVector(gravity);

    // Get the latitude
    double lat;

    ignition::math::Vector3d spherical = this->sphericalCoordinates->SphericalFromLocal(insPose.Pos());
    lat = spherical.X();

    // fill in the message
    msgs::Set(this->msg.mutable_stamp(), this->world->GetSimTime());
    this->msg.set_entity_name(this->Name());
    this->msg.set_roll_deg(insPose.Rot().Roll());
    this->msg.set_pitch_deg(insPose.Rot().Pitch());
    this->msg.set_heading_deg(insPose.Rot().Yaw());
    msgs::Set(this->msg.mutable_orientation(), insPose.Rot());
    msgs::Set(this->msg.mutable_angular_velocity(), angular_velocity);
    msgs::Set(this->msg.mutable_linear_velocity(),  linear_velocity);
    msgs::Set(this->msg.mutable_linear_accel(),  linear_accel);
    this->msg.set_latitude_deg(lat);

    // Actually publish
    if (this->insPub) {
        this->insPub->Publish(this->msg);
    }

    return true;
}

common::Time DsrosInsSensor::GetTime() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::Convert(this->msg.stamp());
}

std::string DsrosInsSensor::GetEntityName() const {
    std::lock_guard<std::mutex>(this->mutex);
    return this->msg.entity_name();
}
    
ignition::math::Quaterniond DsrosInsSensor::GetOrientation() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::ConvertIgn(this->msg.orientation());
}

ignition::math::Vector3d DsrosInsSensor::GetAngularVelocity() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::ConvertIgn(this->msg.angular_velocity());
}

ignition::math::Vector3d DsrosInsSensor::GetLinearVelocity() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::ConvertIgn(this->msg.linear_velocity());
}

ignition::math::Vector3d DsrosInsSensor::GetLinearAcceleration() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::ConvertIgn(this->msg.linear_accel());
}

double DsrosInsSensor::GetLatitude() const {
    std::lock_guard<std::mutex>(this->mutex);
    return this->msg.latitude_deg();
}

double DsrosInsSensor::GetRoll() const {
    std::lock_guard<std::mutex>(this->mutex);
    return this->msg.roll_deg();
}

double DsrosInsSensor::GetPitch() const {
    std::lock_guard<std::mutex>(this->mutex);
    return this->msg.pitch_deg();
}

double DsrosInsSensor::GetHeading() const {
    std::lock_guard<std::mutex>(this->mutex);
    return this->msg.heading_deg();
}

GZ_REGISTER_STATIC_SENSOR("dsros_ins", DsrosInsSensor);

