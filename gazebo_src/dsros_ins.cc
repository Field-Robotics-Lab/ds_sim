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


#include "depth_util.hpp"

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
        gzdbg <<"Sensor " <<this->Name() <<" could not find parent link!" <<std::endl;
        if (parentEntity) {
            gzdbg <<"parent: " <<parentEntity->GetName() <<std::endl;
        }
    }

    // setup to publish!
    this->topicName = this->GetTopic();
    this->insPub = this->node->Advertise<ds_sim::msgs::Ins>(this->topicName, 50);

    // TODO: Add noise!
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
    ignition::math::Pose3d insPose = this->pose + this->parentLink->GetWorldPose().Ign();
    ignition::math::Vector3<double> angular_velocity
         = insPose.Rot().Inverse().RotateVector(this->parentLink->GetWorldAngularVel().Ign());

    math::Vector3 linear_velocity = this->parentLink->GetWorldLinearVel(this->pose.Pos(), this->pose.Rot());

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
    msgs::Set(this->msg.mutable_linear_velocity(),  linear_velocity.Ign());
    this->msg.set_latitude_deg(lat);

    // Actually publish
    if (this->insPub) {
        this->insPub->Publish(this->msg);
    }

    return true;
}

GZ_REGISTER_STATIC_SENSOR("dsros_ins", DsrosInsSensor);

