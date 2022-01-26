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
 * dsros_dvl.hh
 *
 * Ian Vaughn, 2017 Nov 30
 * 
 * A DVL sensor.  This is NOT a plugin, its a whole
 * new sensor type.
 *
 * Loading it requires the epic dsros_sensors SystemPlugin hack.
 */

#pragma once

#include <string>
#include <mutex>
#include <cfloat>

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/common.hh>

#include <SensorDvl.pb.h>
#include <dave_gazebo_world_plugins/StratifiedCurrentVelocity.pb.h>

namespace gazebo {
namespace sensors {

  typedef const boost::shared_ptr<
      const dave_gazebo_world_plugins_msgs::msgs::StratifiedCurrentVelocity>
          ConstStratifiedCurrentVelocityPtr;


  // The gazebo implementation uses pimpl, but we can 
  // easily recompile our plugin, so don't bother.

  // forward declaration
  class DsrosDvlSensor;

  class DsrosDvlBeam {
      public:
        static constexpr double NO_VELOCITY = DBL_MAX;  // "No current velocity" flag value

        DsrosDvlBeam(const physics::PhysicsEnginePtr& physicsEngine,
                  const DsrosDvlSensor* parent, int beamnum,
                  const ignition::math::Pose3d& sensor_pose,
                  const ignition::math::Pose3d& beam_pose);
                

        void Update(const physics::WorldPtr& world, const ignition::math::Vector3d& sensorVel,
                                                    const ignition::math::Vector3d& sensorWtrVel,
                                                    const ignition::math::Pose3d& inst2world);
        const DsrosDvlSensor* parent;
        physics::CollisionPtr collision;
        physics::RayShapePtr shape;
        ignition::math::Pose3d centerPose;

        // thing we contact
        std::string contactEntityName;
        // cache set to the current contactEntity
        physics::EntityPtr contactEntityPtr;

        double contactRange;
        double startRange;
        double beamVelocity;
        double beamWaterVelocity;

        std::vector<double> beamWaterVelocityBins;
        ignition::math::Vector3d beamUnitVector; // in body coordinates

        bool isValid() const;
        int number;
  };

  class DsrosDvlSensor : public Sensor {
    public: DsrosDvlSensor();
    public: virtual ~DsrosDvlSensor();
    public: virtual void Load(const std::string &_worldName,
                              sdf::ElementPtr _sdf);


    public: virtual void Load(const std::string &_worldName);

    public: virtual std::string GetTopic() const;

    public: virtual std::string GetOceanCurrentTopic() const;

    public: virtual std::string GetStratifiedOceanCurrentTopic() const;

    public: virtual void Init();

    public: virtual void Fini();

    // accessors
    public: common::Time GetTime() const;
    public: std::vector<double> GetRanges() const;
    public: ignition::math::Vector3d GetLinearVelocity() const;
    public: ignition::math::Vector3d GetBeamUnitVec(int idx) const;
    public: bool BeamValid(int idx) const;
    public: double GetBeamVelocity(int idx) const;
    public: double GetBeamWaterVelocity(int idx) const;
    public: double GetBeamWaterVelocityBin(int idx, int bin) const;
    public: double GetBeamRange(int idx) const;

    public: double RangeMin() const;
    public: double RangeMax() const;
    public: double RangeDiffMax() const;
    public: double BeamWidth() const;
    public: double BeamAngle() const;
    public: ignition::math::Pose3d GetBeamPose(int idx) const;
    public: size_t NumBeams() const;
    public: int ValidBeams() const;

    public: friend class DsrosDvlBeam;
    protected: virtual bool UpdateImpl(const bool _force);
    protected: void OnOceanCurrent(ConstVector3dPtr &_msg);
    protected: void OnStratifiedOceanCurrent(ConstStratifiedCurrentVelocityPtr &_msg);

    private: ignition::math::Vector3d OceanCurrentAtDepth(double depth) const;

    protected:
        physics::LinkPtr parentLink;
        mutable std::mutex mutex;
    
        transport::PublisherPtr dvlPub;
        std::string topicName;
        ds_sim::msgs::Dvl msg;

        double rangeMin;
        double rangeMax;
        double rangeDiffMax;
        double beamWidth;
        double beamAngle;

        // Azimuth of each beam in DVL coordinates
        double beamAzimuth1;
        double beamAzimuth2;
        double beamAzimuth3;
        double beamAzimuth4;
        bool pos_z_down;

        std::vector<DsrosDvlBeam> beams;

        // Water track/current profiling bin information
        int waterTrackBins;
        double currentProfileCellDepth;
        double currentProfileBin0Distance;

        // Collision physics stuff
        physics::CollisionPtr rangeCollision;
        physics::MeshShapePtr beamShape;
        transport::SubscriberPtr contactSub;

        // Ocean current subscription (if it's being published)
        std::string currentTopicName;
        transport::SubscriberPtr currentSub;
        ignition::math::Vector3d oceanCurrent;
        bool recvdOceanCurrent = false;  // received at least 1 flag

        // Stratified ocean current subscription (if it's being published)
        std::string stratifiedCurrentTopicName;
        transport::SubscriberPtr stratifiedCurrentSub;
        std::vector<ignition::math::Vector4d> stratifiedOceanCurrent;
        bool recvdStratifiedOceanCurrent = false;  // received at least 1 flag
  }; // class declaration
}; // namespace sensors
}; // namespace gazebo

// created with the GZ_REGISTER_STATIC_SENSOR macro
extern void RegisterDsrosDvlSensor();

