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

#include "dsros_dvl.hh"

#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <limits>
#include <iomanip>


using namespace gazebo;
using namespace sensors;

DsrosDvlBeam::DsrosDvlBeam(const physics::PhysicsEnginePtr& physicsEngine,
                  const DsrosDvlSensor* parent, int beamnum,
                  const ignition::math::Pose3d& sensor_pose,
                  const ignition::math::Pose3d& beam_pose) {

    // Compute the center pose
    double range = parent->RangeMax();
    double radius = range * tan(parent->BeamWidth()*M_PI/180.0);
    startRange = parent->RangeMin();
    //ignition::math::Pose3d beam_tform = beam_pose + sensor_pose;
    //ignition::math::Vector3d offset(0,0,range*0.5);
    //offset = beam_tform.Rot().RotateVector(offset);
    //this->centerPose.Set(beam_tform.Pos() - offset, beam_tform.Rot());
    this->centerPose = beam_pose + sensor_pose;
    ignition::math::Vector3d startPt 
            = this->centerPose.Rot().RotateVector(ignition::math::Vector3d(0,0,parent->RangeMin())) + this->centerPose.Pos();
    ignition::math::Vector3d endPt 
            = this->centerPose.Rot().RotateVector(ignition::math::Vector3d(0,0,parent->RangeMax())) + this->centerPose.Pos();

    //gzdbg <<"Initializing DVL beam #" <<beamnum <<" with range " <<range <<" radius " <<radius <<"\n";

    this->beamUnitVector = beam_pose.Rot().RotateVector(ignition::math::Vector3d(0,0,1.0));

    std::stringstream cvt;
    cvt <<beamnum;
    std::string beamname = cvt.str();

    this->number = beamnum;
    this->collision = physicsEngine->CreateCollision("ray", parent->ParentName());
    GZ_ASSERT(this->collision != NULL, "Unable to create a multiray collision using physics engine!");

    this->collision->SetName(parent->ScopedName() + "dvl_sensor_collision_" + beamname);
    this->collision->AddType(physics::Base::SENSOR_COLLISION);
    parent->parentLink->AddChild(this->collision);
    this->collision->SetRelativePose(this->centerPose);
    this->collision->SetInitialRelativePose(this->centerPose);

    this->shape = boost::dynamic_pointer_cast<physics::RayShape>(this->collision->GetShape());
    GZ_ASSERT(this->shape != NULL, "Unable to get beam shape for DVL beam!");
    //this->shape->SetMesh("unit_cone");
    //this->shape->SetScale(ignition::math::Vector3d(
            //radius*2.0, radius*2.0, range));
    this->shape->SetPoints(startPt, endPt);
    this->shape->Init();

    this->collision->GetSurface()->collideWithoutContact = true;
    this->collision->GetSurface()->collideWithoutContactBitmask = 1;
    this->collision->SetCollideBits(GZ_SENSOR_COLLIDE);
    this->collision->SetCategoryBits(GZ_SENSOR_COLLIDE);

    // setup the contact subscription
    /*
    std::string topic = 
       physicsEngine->GetContactManager()->CreateFilter(this->collision->GetScopedName(), 
                                                        this->collision->GetScopedName());
    this->contactSub = parent->node->Subscribe(topic, &DsrosDvlBeam::OnContacts, this);
    */
}

bool DsrosDvlBeam::isValid() const {
    return !contactEntityName.empty();
}

void DsrosDvlBeam::Update(const physics::WorldPtr& world, 
                          const ignition::math::Vector3d& sensorVel, 
                          const ignition::math::Pose3d& inst2world) {

    
    // update the beam intersections
    std::string entityName;
    this->shape->Update();
    this->shape->GetIntersection(contactRange, entityName);
    contactRange += startRange; // have to add the initial range offset.  #bugfix.

    if (entityName.empty()) {
        contactEntityName = "";
        contactEntityPtr.reset();
        contactRange = -1;
        beamVelocity = 0;
        return;
    }

    // (possibly) update the contact cache
    if (entityName != contactEntityName) {
        contactEntityName = entityName;
        contactEntityPtr = world->GetEntity(entityName);
    }

    // get velocities
    if (contactEntityPtr.get() != NULL) {
        // Compute the intersection in world coordinates
        ignition::math::Vector3d intersection = inst2world.Rot().RotateVector(contactRange * beamUnitVector) + inst2world.Pos();
        
        // Compute the velocity of the point of impact in world coordinates
        ignition::math::Vector3d entityVel = contactEntityPtr->GetWorldLinearVel().Ign()
            + contactEntityPtr->GetWorldAngularVel().Ign().Cross(
                    contactEntityPtr->GetWorldPose().pos.Ign() - intersection);
        //gzdbg <<"ENTITY: " <<entityVel.X() <<", " <<entityVel.Y() <<", " <<entityVel.Z() <<"\n";

        //gzdbg <<"intersection: " <<intersection.X() <<","  <<intersection.Y() <<","  <<intersection.Z() <<"\n";
        ignition::math::Vector3d inst_rel_vel = inst2world.Rot().RotateVectorReverse(sensorVel - entityVel);
        //gzdbg <<"\tinst_relative vel: " <<inst_rel_vel.X() <<","  <<inst_rel_vel.Y() <<","  <<inst_rel_vel.Z() <<"\n";
        //gzmsg <<"\tunit vector: " <<beamUnitVector.X() <<"," <<beamUnitVector.Y() <<"," <<beamUnitVector.Z() <<"\n";
        beamVelocity = beamUnitVector.Dot(inst_rel_vel);
    } else {
        gzdbg <<"NULL contact entity!";
    }
}

DsrosDvlSensor::DsrosDvlSensor() : Sensor(sensors::OTHER){
    // do nothing
};

DsrosDvlSensor::~DsrosDvlSensor() {
    // do nothing
}

void DsrosDvlSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf) {
    Sensor::Load(_worldName, _sdf);
} 

void DsrosDvlSensor::Load(const std::string &_worldName) {
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

    // Load the options
    sdf::ElementPtr dvlElem = this->sdf->GetFirstElement();

    while (dvlElem) {
      gzdbg <<dvlElem->GetName() <<"\n";

      if (dvlElem->GetName() == "plugin"
        && dvlElem->GetAttribute("filename")->GetAsString() == "libdsros_ros_dvl.so") {
          break;

      }
      dvlElem = dvlElem->GetNextElement();
    }

    if (!dvlElem) {
      gzerr <<"DVL sensor MUST have plugin with filename=libdsros_ros_dvl.so so we can get min/max range, etc\n";
      throw std::runtime_error("DVL Sensor MUST specify a plugin with parameters");
    }

    this->rangeMin     = dvlElem->Get<double>("minRange");
    this->rangeMax     = dvlElem->Get<double>("maxRange");
    this->rangeDiffMax = dvlElem->Get<double>("maxRangeDiff");
    this->beamAngle    = dvlElem->Get<double>("beamAngleDeg");
    this->beamWidth    = dvlElem->Get<double>("beamWidthDeg");
    this->beamAzimuth1  = dvlElem->Get<double>("beamAzimuthDeg1");
    this->beamAzimuth2  = dvlElem->Get<double>("beamAzimuthDeg2");
    this->beamAzimuth3  = dvlElem->Get<double>("beamAzimuthDeg3");
    this->beamAzimuth4  = dvlElem->Get<double>("beamAzimuthDeg4");
    this->pos_z_down = dvlElem->Get<bool>("pos_z_down");
    /*
    this->rangeMin = 1.0;
    this->rangeMax = 200;
    this->rangeDiffMax = 10;
    this->beamAngle = 30;
    this->beamWidth = 4.0;
     */

    // setup to publish!
    this->topicName = this->GetTopic();
    this->dvlPub = this->node->Advertise<ds_sim::msgs::Dvl>(this->topicName, 50);

    // Setup physics!
    physics::PhysicsEnginePtr physicsEngine = this->world->GetPhysicsEngine();
    GZ_ASSERT(physicsEngine != NULL, "Unable to get pointer to physics engine");

    // rotate 
    ignition::math::Pose3d beamPose;

    const double DTOR = M_PI/180.0;

    double start = M_PI;
    if (this->pos_z_down) {
        start = 0;
    }
    
    // RDI has this really silly beam arrangement
    beamPose.Set(0,0,0, 0, (start-DTOR*(this->beamAngle)), this->beamAzimuth1*DTOR);
    beams.push_back(DsrosDvlBeam(physicsEngine, this, 1, this->pose, beamPose));

    beamPose.Set(0,0,0, 0, (start-DTOR*(this->beamAngle)), this->beamAzimuth2*DTOR);
    beams.push_back(DsrosDvlBeam(physicsEngine, this, 2, this->pose, beamPose));

    beamPose.Set(0,0,0, 0, (start-DTOR*(this->beamAngle)), this->beamAzimuth3*DTOR);
    beams.push_back(DsrosDvlBeam(physicsEngine, this, 3, this->pose, beamPose));

    beamPose.Set(0,0,0, 0, (start-DTOR*(this->beamAngle)), this->beamAzimuth4*DTOR);
    beams.push_back(DsrosDvlBeam(physicsEngine, this, 4, this->pose, beamPose));
}

void DsrosDvlSensor::Init() {
    Sensor::Init();
}

void DsrosDvlSensor::Fini() {

    parentLink.reset();
    Sensor::Fini();
}

std::string DsrosDvlSensor::GetTopic() const {
    std::string topicName = "~/" + this->ParentName() + '/' + this->Name();

    if (this->sdf->HasElement("topic")) {
        topicName += '/' + this->sdf->Get<std::string>("topic");
    } else {
        topicName += "/dvl";
    }

    boost::replace_all(topicName, "::", "/");

    return topicName;
}

common::Time DsrosDvlSensor::GetTime() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::Convert(this->msg.stamp());
}

std::vector<double> DsrosDvlSensor::GetRanges() const {
    std::lock_guard<std::mutex>(this->mutex);
    std::vector<double> ret(this->msg.ranges_size());
    for(size_t i=0; i<ret.size(); i++) {
        ret[i] = msg.ranges(i);
    }
    return ret;
}

ignition::math::Vector3d DsrosDvlSensor::GetLinearVelocity() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msgs::ConvertIgn(this->msg.linear_velocity());
}

ignition::math::Vector3d DsrosDvlSensor::GetBeamUnitVec(int idx) const {
    std::lock_guard<std::mutex>(this->mutex);
    return beams[idx].beamUnitVector;
}

double DsrosDvlSensor::RangeMin() const {
    return rangeMin;
}
double DsrosDvlSensor::RangeMax() const {
    return rangeMax;
}
double DsrosDvlSensor::RangeDiffMax() const {
    return rangeDiffMax;
}
double DsrosDvlSensor::BeamWidth() const {
    return beamWidth;
}
double DsrosDvlSensor::BeamAngle() const {
    return beamAngle;
}

ignition::math::Pose3d DsrosDvlSensor::GetBeamPose(int idx) const {
    return beams[idx].centerPose;
}

int DsrosDvlSensor::ValidBeams() const {
    std::lock_guard<std::mutex>(this->mutex);
    return msg.num_beams();
}

size_t DsrosDvlSensor::NumBeams() const {
    return beams.size();
}

bool DsrosDvlSensor::BeamValid(int idx) const {
    return beams[idx].isValid();
}

double DsrosDvlSensor::GetBeamVelocity(int idx) const {
    return beams[idx].beamVelocity;
}

double DsrosDvlSensor::GetBeamRange(int idx) const {
    return beams[idx].contactRange;
}

bool DsrosDvlSensor::UpdateImpl(const bool _force) {

    if (!this->parentLink) {
        return true;
    }

    // Acquire a mutex to avoid a race condition
    boost::recursive_mutex::scoped_lock engine_lock(*(
        this->world->GetPhysicsEngine()->GetPhysicsUpdateMutex()));

    std::lock_guard<std::mutex> lock(this->mutex);

    // update each beam
    ignition::math::Pose3d vehPose = this->parentLink->GetWorldPose().Ign();
    ignition::math::Vector3d bodyLinearVel = this->parentLink->GetWorldLinearVel().Ign();
    ignition::math::Vector3d bodyAngularVel = this->parentLink->GetWorldAngularVel().Ign();
    ignition::math::Vector3d sensorVel = bodyLinearVel + vehPose.Rot().RotateVector(bodyAngularVel.Cross(this->pose.Pos()));
    ignition::math::Pose3d sensorPose = this->pose + this->parentLink->GetWorldPose().Ign();

    // Compute a solution for all beams
    int valid_beams = 0;
    int basis_fill_in = 0;
    Eigen::Matrix<double, Eigen::Dynamic, 3> beam_basis(4,3);
    Eigen::VectorXd beam_vel(4);
    for (size_t i=0; i<beams.size(); i++) {
        beams[i].Update(world, sensorVel, sensorPose);
        //msg <<beams[i].contactRange <<"/" <<beams[i].beamVelocity <<"  ";
        if (beams[i].isValid()) {
            valid_beams++;
            beam_basis(basis_fill_in,0) = beams[i].beamUnitVector.X();
            beam_basis(basis_fill_in,1) = beams[i].beamUnitVector.Y();
            beam_basis(basis_fill_in,2) = beams[i].beamUnitVector.Z();
            beam_vel(basis_fill_in) = beams[i].beamVelocity;
            basis_fill_in++;
        }
    }
    //gzdbg <<msg.str() <<" valid: " <<valid_beams <<"\n";

    // Compute the velocity solution
    ignition::math::Vector3d linear_velocity;
    if (valid_beams >= 3) {
        beam_basis = beam_basis.topRows(valid_beams);
        beam_vel = beam_vel.head(valid_beams);

        // we're going to solve a least-squares problem:
        // beam_basis.transpose() * instrument_vel = beam_vel
        // For some reason this insists on a fully-dynamic matrix
        Eigen::MatrixXd H(beam_basis);
        Eigen::Vector3d inst_vel = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beam_vel);

        linear_velocity.X( inst_vel(0) );
        linear_velocity.Y( inst_vel(1) );
        linear_velocity.Z( inst_vel(2) );

        ignition::math::Vector3d stefVel = sensorPose.Rot().RotateVector(linear_velocity);
        ignition::math::Vector3d bodyVel = vehPose.Rot().RotateVectorReverse(bodyLinearVel);

        //gzdbg <<" comp. vel: (" <<inst_vel(0) <<"," <<inst_vel(1) <<"," <<inst_vel(2) <<")\n"
        //      <<" orig. vel: (" <<bodyVel.X() <<"," <<bodyVel.Y() <<"," <<bodyVel.Z() <<")\n"
        //      <<" stef. vel: (" <<stefVel.X() <<"," <<stefVel.Y() <<"," <<stefVel.Z() <<")\n";
    }

    // fill in the message
    msgs::Set(this->msg.mutable_stamp(), this->world->GetSimTime());
    msgs::Set(this->msg.mutable_linear_velocity(), linear_velocity);
    this->msg.set_num_beams(valid_beams);
    for (size_t i=0; i<msg.ranges_size(); i++) {
        if (beams[i].isValid()) {
            this->msg.set_ranges(i, beams[i].contactRange);
            this->msg.set_range_velocities(i, beams[i].beamVelocity);
        } else {
            this->msg.set_ranges(i, std::numeric_limits<double>::quiet_NaN());
            this->msg.set_range_velocities(i, std::numeric_limits<double>::quiet_NaN());
        }
        msgs::Set(this->msg.mutable_unit_vectors(i), beams[i].beamUnitVector);
    }

    // Actually publish
    if (this->dvlPub) {
        this->dvlPub->Publish(this->msg);
    }

    return true;
}

GZ_REGISTER_STATIC_SENSOR("dsros_dvl", DsrosDvlSensor);

