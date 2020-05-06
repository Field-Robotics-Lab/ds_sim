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
 * dsros_depthsensor.cc
 *
 * Ian Vaughn, 2017 Nov 16
 * 
 * A depth sensor.  This is NOT a plugin, its a whole
 * new sensor type.
 *
 * Loading it requires the epic dsros_sensors SystemPlugin hack.
 */

#include "dsros_depth.hh"

#include <boost/algorithm/string.hpp>


#include "depth_util.hpp"

using namespace gazebo;
using namespace sensors;

DsrosDepthSensor::DsrosDepthSensor() : Sensor(sensors::OTHER){
    // do nothing
};

DsrosDepthSensor::~DsrosDepthSensor() {
    // do nothing
}

void DsrosDepthSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf) {
    Sensor::Load(_worldName, _sdf);
} 

void DsrosDepthSensor::Load(const std::string &_worldName) {
    Sensor::Load(_worldName);

    // Load the parent link
#if GAZEBO_MAJOR_VERSION > 7
    physics::EntityPtr parentEntity = this->world->EntityByName(this->ParentName());
#else
  physics::EntityPtr parentEntity = this->world->GetEntity(this->ParentName());
#endif
    this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);
    if (! this->parentLink) {
        gzdbg <<"Sensor " <<this->Name() <<" could not find parent link!" <<std::endl;
        if (parentEntity) {
            gzdbg <<"parent: " <<parentEntity->GetName() <<std::endl;
        }
    }

    // setup to publish!
    this->topicName = this->GetTopic();
    this->depthPub = this->node->Advertise<ds_sim::msgs::PressureDepth>(this->topicName, 50);

    // TODO: Add noise!
}

void DsrosDepthSensor::Init() {
    Sensor::Init();
#if GAZEBO_MAJOR_VERSION > 7
    this->sphericalCoordinates = this->world->SphericalCoords();
#else
  this->sphericalCoordinates = this->world->GetSphericalCoordinates();
#endif
}

void DsrosDepthSensor::Fini() {
    Sensor::Fini();
    parentLink.reset();
}

std::string DsrosDepthSensor::GetTopic() const {
    std::string topicName = "~/" + this->ParentName() + '/' + this->Name();

    if (this->sdf->HasElement("topic")) {
        topicName += '/' + this->sdf->Get<std::string>("topic");
    } else {
        topicName += "/depth";
    }

    boost::replace_all(topicName, "::", "/");

    return topicName;
}

common::Time DsrosDepthSensor::GetTime() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::Convert(this->msg.stamp());
}

double DsrosDepthSensor::GetDepth() const {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->msg.depth();
}

double DsrosDepthSensor::GetPressure() const {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->msg.pressure_dbar();
}

double DsrosDepthSensor::GetLatitude() const {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->msg.latitude_deg();
}

bool DsrosDepthSensor::UpdateImpl(const bool _force) {
    std::lock_guard<std::mutex> lock(this->mutex);

    if (!this->parentLink) {
        return true;
    }

    // Get the actual depth
    double raw_depth;
#if GAZEBO_MAJOR_VERSION > 7
    ignition::math::Pose3d depthPose = this->pose + this->parentLink->WorldPose();
#else
    ignition::math::Pose3d depthPose = this->pose + this->parentLink->GetWorldPose().Ign();
#endif

    raw_depth = -depthPose.Pos().Z();

    // Get the latitude
    double lat, press;

    ignition::math::Vector3d spherical = this->sphericalCoordinates->SphericalFromLocal(depthPose.Pos());
    lat = spherical.X();

    // Do the pressure inversion
    press = fofonoff_pressure(raw_depth, lat);

    // Do a FORWARD Fofonoff depth calculation so the data 
    // looks consistent
    // Basically, this guarantees that we can use the pressure + latitude in a depth
    // sample to get the same depth value, so it all looks less confusing later.
    // The difference between the recomputed depth and the actual depth
    // should be < 5e-5m, which is in the noise.
    double depth = fofonoff_depth(press, lat);

    //gzdbg <<"Raw Depth: " <<raw_depth << " DEPTH: " <<depth 
    //      <<" ERR: " <<depth - raw_depth <<" LAT: " <<lat <<" PRESS: " << press <<"\n";

    // fill in the message
#if GAZEBO_MAJOR_VERSION > 7
    msgs::Set(this->msg.mutable_stamp(), this->world->SimTime());
#else
    msgs::Set(this->msg.mutable_stamp(), this->world->GetSimTime());
#endif
    this->msg.set_depth(depth);
    this->msg.set_pressure_dbar(press);
    this->msg.set_latitude_deg(lat);

    // Actually publish
    if (this->depthPub) {
        this->depthPub->Publish(this->msg);
    }

    return true;
}

GZ_REGISTER_STATIC_SENSOR("dsros_depth", DsrosDepthSensor);

