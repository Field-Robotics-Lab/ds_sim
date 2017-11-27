/*
 *
 * dsros_depthsensor.hh
 *
 * Ian Vaughn, 2017 Nov 22
 * 
 * A depth sensor.  This is NOT a plugin, its a whole
 * new sensor type.
 *
 * Loading it requires the epic dsros_sensors SystemPlugin hack.
 */

#pragma once

#include <string>
#include <mutex>

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <SensorIns.pb.h>

namespace gazebo {
namespace sensors {

  // The gazebo implementation uses pimpl, but we can 
  // easily recompile our plugin, so don't bother.

  class DsrosInsSensor : public Sensor {
    public: DsrosInsSensor();
    public: virtual ~DsrosInsSensor();
    public: virtual void Load(const std::string &_worldName,
                              sdf::ElementPtr _sdf);


    public: virtual void Load(const std::string &_worldName);

    public: virtual std::string GetTopic() const;

    public: virtual void Init();

    public: virtual void Fini();

    protected: virtual bool UpdateImpl(const bool _force);

    protected:
        physics::LinkPtr parentLink;
        mutable std::mutex mutex;
    
        common::SphericalCoordinatesPtr sphericalCoordinates;
        transport::PublisherPtr insPub;
        std::string topicName;
        ds_sim::msgs::Ins msg;
  }; // class declaration
}; // namespace sensors
}; // namespace gazebo

// created with the GZ_REGISTER_STATIC_SENSOR macro
extern void RegisterDsrosInsSensor();


