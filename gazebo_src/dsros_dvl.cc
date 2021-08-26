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

    this->parent = parent;

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

    this->beamWaterVelocityBins.resize(parent->waterTrackBins);

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
                          const ignition::math::Vector3d& sensorWtrVel,
                          const ignition::math::Pose3d& inst2world) {

    
    // update the beam intersections
    std::string entityName;
    this->shape->Update();
    this->shape->GetIntersection(contactRange, entityName);
    contactRange += startRange; // have to add the initial range offset.  #bugfix.

    // if no bottom return, use sensor water velocity to compute the beam's water track velocity
    // NOTE: This is based on the "global" current, not the stratified current
    ignition::math::Vector3d inst_wtr_vel = inst2world.Rot().RotateVectorReverse(sensorWtrVel);
    beamWaterVelocity = beamUnitVector.Dot(inst_wtr_vel);

    // Use stratified current to compute bean-specific velocities for each bin
    for (int bin = 0; bin < this->beamWaterVelocityBins.size(); bin++)
    {
        double binRange = this->parent->currentProfileBin0Distance +
                          bin * this->parent->currentProfileCellDepth;
        if ((this->contactRange > 0) && (binRange > this->contactRange))
        {
            this->beamWaterVelocityBins[bin] = DsrosDvlBeam::NO_VELOCITY;
        }
        else
        {
            ignition::math::Vector3d binPt =
                inst2world.Rot().RotateVector(binRange * beamUnitVector) +
                inst2world.Pos();
            ignition::math::Vector3d current =
                this->parent->OceanCurrentAtDepth(-binPt.Z());
            ignition::math::Vector3d binVelocity =
                inst2world.Rot().RotateVectorReverse(sensorVel - current);
            this->beamWaterVelocityBins[bin] = this->beamUnitVector.Dot(binVelocity);
//            gzmsg << "Beam " << this->number << " bin " << bin
//                  << " water velocity info:" << std::endl;
//            gzmsg << "  Bin depth: " << binRange << std::endl;
//            gzmsg << "  Bin point: " << binPt << std::endl;
//            gzmsg << "  Current at bin point depth: " << current << std::endl;
//            gzmsg << "  Bin velocity: " << binVelocity << " and value: "
 //                 << this->beamWaterVelocityBins[bin] << std::endl;
        }
    }

    if (entityName.empty()) {
        contactEntityName = "";
        contactEntityPtr.reset();
        contactRange = -1;
        return;
    }

    // (possibly) update the contact cache
    if (entityName != contactEntityName) {
        contactEntityName = entityName;
#if GAZEBO_MAJOR_VERSION > 7
        contactEntityPtr = world->EntityByName(entityName);
#else
        contactEntityPtr = world->GetEntity(entityName);
#endif
    }

  // get velocities
  if (contactEntityPtr.get() != NULL) {
#if GAZEBO_MAJOR_VERSION > 7
      ignition::math::Vector3d contactWorldPos = contactEntityPtr->WorldPose().Pos();
      ignition::math::Vector3d contactLinearVel = contactEntityPtr->WorldLinearVel();
      ignition::math::Vector3d contactAngularVel = contactEntityPtr->WorldAngularVel();
#else
      ignition::math::Vector3d contactWorldPos = contactEntityPtr->GetWorldPose().pos.Ign();
      ignition::math::Vector3d contactLinearVel = contactEntityPtr->GetWorldLinearVel().Ign();
      ignition::math::Vector3d contactAngularVel = contactEntityPtr->GetWorldAngularVel().Ign();
#endif

      // Compute the intersection in world coordinates
      ignition::math::Vector3d intersection = inst2world.Rot().RotateVector(contactRange * beamUnitVector) + inst2world.Pos();
        
      // Compute the velocity of the point of impact in world coordinates
      ignition::math::Vector3d entityVel = contactLinearVel + contactAngularVel.Cross(contactWorldPos - intersection);
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
      //gzdbg <<"DVL sensor MUST have plugin with filename=libdsros_ros_dvl.so so we can get min/max range, etc\n";
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

    // Set up water tracking/current profiling bins
    if (dvlElem->HasElement("waterTrackBins"))
    {
        this->waterTrackBins = dvlElem->Get<int>("waterTrackBins");
//        gzmsg << "Water track bins set to " << this->waterTrackBins << std::endl;
    }
    else
    {
        this->waterTrackBins = 1;
//        gzmsg << "Water track bins set to default" << std::endl;
    }
    this->currentProfileCellDepth = (this->rangeMax - this->rangeMin) /
                                    this->waterTrackBins;
    this->currentProfileBin0Distance = this->rangeMin +
                                       this->currentProfileCellDepth / 2.0;

    // setup to publish!
    this->topicName = this->GetTopic();
    this->dvlPub = this->node->Advertise<ds_sim::msgs::Dvl>(this->topicName, 50);

    // Setup physics!
    physics::PhysicsEnginePtr physicsEngine;
#if GAZEBO_MAJOR_VERSION > 7
    physicsEngine = this->world->Physics();
#else
    physicsEngine = this->world->GetPhysicsEngine();
#endif
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

    // setup the ocean current subscription
    this->currentTopicName = this->GetOceanCurrentTopic();
    this->currentSub = this->node->Subscribe(this->currentTopicName, 
                                             &DsrosDvlSensor::OnOceanCurrent, this);

    // setup the stratified ocean current subscription
    this->stratifiedCurrentTopicName = this->GetStratifiedOceanCurrentTopic();
    this->stratifiedCurrentSub =
        this->node->Subscribe(this->stratifiedCurrentTopicName,
                              &DsrosDvlSensor::OnStratifiedOceanCurrent, this);
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

std::string DsrosDvlSensor::GetOceanCurrentTopic() const {
    std::string topicName = "";

    if (this->sdf->HasElement("ocean_current_topic")) {
        topicName += this->sdf->Get<std::string>("ocean_current_topic");
    } else {
        topicName += "hydrodynamics/current_velocity";
    }

    boost::replace_all(topicName, "::", "/");

    return topicName;
}

std::string DsrosDvlSensor::GetStratifiedOceanCurrentTopic() const {
    std::string topicName = "";

    if (this->sdf->HasElement("stratified_ocean_current_topic")) {
        topicName += this->sdf->Get<std::string>("stratified_ocean_current_topic");
    } else {
        topicName += "hydrodynamics/stratified_current_velocity";
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

double DsrosDvlSensor::GetBeamWaterVelocity(int idx) const {
    return beams[idx].beamWaterVelocity;
}

double DsrosDvlSensor::GetBeamWaterVelocityBin(int idx, int bin) const {
    return beams[idx].beamWaterVelocityBins[bin];
}

double DsrosDvlSensor::GetBeamRange(int idx) const {
    return beams[idx].contactRange;
}

void DsrosDvlSensor::OnOceanCurrent(ConstVector3dPtr &_msg) {
    this->recvdOceanCurrent = true;
    this->oceanCurrent.X() = _msg->x();
    this->oceanCurrent.Y() = _msg->y();
    this->oceanCurrent.Z() = _msg->z();
//    gzmsg << "Update ocean current to " << this->oceanCurrent << "\n";
}

void DsrosDvlSensor::OnStratifiedOceanCurrent(
  ConstStratifiedCurrentVelocityPtr &_msg) {
    this->recvdStratifiedOceanCurrent = true;
    stratifiedOceanCurrent.clear();
    double xCurrent = 0.0;
    double yCurrent = 0.0;
    double zCurrent = 0.0;
    for (int i=0; i < _msg->velocity_size(); i++) {
        ignition::math::Vector4d vel(_msg->velocity(i).x(),
                                     _msg->velocity(i).y(),
                                     _msg->velocity(i).z(),
                                     _msg->depth(i));
        this->stratifiedOceanCurrent.push_back(vel);
        xCurrent += _msg->velocity(i).x();
        yCurrent += _msg->velocity(i).y();
        zCurrent += _msg->velocity(i).z();
//        gzmsg << "Stratified current at " << vel.W()
//              << " meters: (" << vel.X() << ", " << vel.Y() << ", "
//              << vel.Z() << ")" << std::endl;
    }
    // use average stratified current for global
    // if no global current received yet
    if (!this->recvdOceanCurrent && 
        (this->stratifiedOceanCurrent.size() > 0)) {
        this->oceanCurrent.X() = 
            xCurrent / this->stratifiedOceanCurrent.size();
        this->oceanCurrent.Y() = 
            yCurrent / this->stratifiedOceanCurrent.size();
        this->oceanCurrent.Z() = 
            zCurrent / this->stratifiedOceanCurrent.size();
    }
}

bool DsrosDvlSensor::UpdateImpl(const bool _force) {

    if (!this->parentLink) {
        return true;
    }

    // Acquire a mutex to avoid a race condition
#if GAZEBO_MAJOR_VERSION > 7
    boost::recursive_mutex::scoped_lock engine_lock(*(
        this->world->Physics()->GetPhysicsUpdateMutex()));
#else
  boost::recursive_mutex::scoped_lock engine_lock(*(
      this->world->GetPhysicsEngine()->GetPhysicsUpdateMutex()));
#endif

    std::lock_guard<std::mutex> lock(this->mutex);

    // update each beam
#if GAZEBO_MAJOR_VERSION > 7
    ignition::math::Pose3d vehPose = this->parentLink->WorldPose();
    ignition::math::Vector3d bodyLinearVel = this->parentLink->WorldLinearVel();
    ignition::math::Vector3d bodyWaterLinearVel = this->parentLink->WorldLinearVel() - this->oceanCurrent;
    ignition::math::Vector3d bodyAngularVel = this->parentLink->WorldAngularVel();
    ignition::math::Pose3d sensorPose = this->pose + this->parentLink->WorldPose();
#else
  ignition::math::Pose3d vehPose = this->parentLink->GetWorldPose().Ign();
  ignition::math::Vector3d bodyLinearVel = this->parentLink->GetWorldLinearVel().Ign();
  ignition::math::Vector3d bodyWaterLinearVel = this->parentLink->GetWorldLinearVel().Ign() - this->oceanCurrent;
  ignition::math::Vector3d bodyAngularVel = this->parentLink->GetWorldAngularVel().Ign();
  ignition::math::Pose3d sensorPose = this->pose + this->parentLink->GetWorldPose().Ign();
#endif
  ignition::math::Vector3d sensorVel = bodyLinearVel + vehPose.Rot().RotateVector(bodyAngularVel.Cross(this->pose.Pos()));
  ignition::math::Vector3d sensorWaterVel = bodyWaterLinearVel + vehPose.Rot().RotateVector(bodyAngularVel.Cross(this->pose.Pos()));

    // Compute a solution for all beams
    int valid_beams = 0;
    int basis_fill_in = 0;
    Eigen::Matrix<double, Eigen::Dynamic, 3> beam_basis(4,3);
    Eigen::VectorXd beam_vel(4);
    Eigen::Matrix<double, Eigen::Dynamic, 3> beam_wtr_basis(4,3);
    Eigen::VectorXd beam_wtr_vel(4);
    for (size_t i=0; i<beams.size(); i++) {
        beams[i].Update(world, sensorVel, sensorWaterVel, sensorPose);
        //msg <<beams[i].contactRange <<"/" <<beams[i].beamVelocity <<"  ";
        if (beams[i].isValid()) {
            valid_beams++;
            beam_basis(basis_fill_in,0) = beams[i].beamUnitVector.X();
            beam_basis(basis_fill_in,1) = beams[i].beamUnitVector.Y();
            beam_basis(basis_fill_in,2) = beams[i].beamUnitVector.Z();
            beam_vel(basis_fill_in) = beams[i].beamVelocity;
            basis_fill_in++;
        }
        // compute in water track values even if no bottom return
        beam_wtr_basis(i,0) = beams[i].beamUnitVector.X();
        beam_wtr_basis(i,1) = beams[i].beamUnitVector.Y();
        beam_wtr_basis(i,2) = beams[i].beamUnitVector.Z();
        beam_wtr_vel(i) = beams[i].beamWaterVelocity;
        //gzdbg << "beam[" << i << "] vel: " << beams[i].beamVelocity <<", wtr vel: " <<beams[i].beamWaterVelocity <<"\n";
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
    } else {
        // same approach for a water track solution, but use the beamWaterVelocity values
        // assumes all beams are "valid" as far as the water track solution goes
        Eigen::MatrixXd H(beam_wtr_basis);
        Eigen::Vector3d inst_vel = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beam_wtr_vel);

        linear_velocity.X( inst_vel(0) );
        linear_velocity.Y( inst_vel(1) );
        linear_velocity.Z( inst_vel(2) );

        ignition::math::Vector3d stefVel = sensorPose.Rot().RotateVector(linear_velocity);
        ignition::math::Vector3d bodyVel = vehPose.Rot().RotateVectorReverse(bodyWaterLinearVel);
        //gzdbg <<" comp. vel: (" <<inst_vel(0) <<"," <<inst_vel(1) <<"," <<inst_vel(2) <<")\n"
        //      <<" orig. vel: (" <<bodyVel.X() <<"," <<bodyVel.Y() <<"," <<bodyVel.Z() <<")\n"
        //      <<" stef. vel: (" <<stefVel.X() <<"," <<stefVel.Y() <<"," <<stefVel.Z() <<")\n";
    }

    // fill in the message
#if GAZEBO_MAJOR_VERSION > 7
    msgs::Set(this->msg.mutable_stamp(), this->world->SimTime());
#else
    msgs::Set(this->msg.mutable_stamp(), this->world->GetSimTime());
#endif
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

ignition::math::Vector3d DsrosDvlSensor::OceanCurrentAtDepth(double depth) const
{
    ignition::math::Vector3d current(0.0, 0.0, 0.0);
    // If test depth is above the surface, just return (0, 0, 0)
    if (depth < 0.0)
    {
        return current;
    }

    // If no stratified current, return the single current value
    if (this->stratifiedOceanCurrent.size() == 0)
    {
        return this->oceanCurrent;
    }

    // Shallower than first depth (return first depth value)
    if (depth <= this->stratifiedOceanCurrent.front().W())
    {
        ignition::math::Vector4d stratCurrent =
            this->stratifiedOceanCurrent.front();
        current.Set(stratCurrent.X(), stratCurrent.Y(), stratCurrent.Z());
    }

    // Deeper than last depth (return last depth value)
    if (depth >= this->stratifiedOceanCurrent.back().W())
    {
        ignition::math::Vector4d stratCurrent =
            this->stratifiedOceanCurrent.front();
        current.Set(stratCurrent.X(), stratCurrent.Y(), stratCurrent.Z());
    }

    double lowIndex = 0;
    ignition::math::Vector4d lower = this->stratifiedOceanCurrent[lowIndex];
    ignition::math::Vector4d upper = this->stratifiedOceanCurrent[lowIndex+1];
    while (depth > upper.W())
    {
        lowIndex++;
        lower = upper;
        upper = this->stratifiedOceanCurrent[lowIndex+1];
    }
    double interp = (depth - lower.W()) / (upper.W() - lower.W());
    double xCurrent = lower.X() + interp * (upper.X() - lower.X());
    double yCurrent = lower.Y() + interp * (upper.Y() - lower.Y());
    double zCurrent = lower.Z() + interp * (upper.Z() - lower.Z());
    current.Set(xCurrent, yCurrent, zCurrent);
    return current;
}

GZ_REGISTER_STATIC_SENSOR("dsros_dvl", DsrosDvlSensor);

