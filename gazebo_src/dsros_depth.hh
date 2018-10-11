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

#include <SensorDepth.pb.h>

namespace gazebo {
namespace sensors {

  // The gazebo implementation uses pimpl, but we can 
  // easily recompile our plugin, so don't bother.

  class DsrosDepthSensor : public Sensor {
    public: DsrosDepthSensor();
    public: virtual ~DsrosDepthSensor();
    public: virtual void Load(const std::string &_worldName,
                              sdf::ElementPtr _sdf);


    public: virtual void Load(const std::string &_worldName);

    public: virtual std::string GetTopic() const;

    public: virtual void Init();

    public: virtual void Fini();

    public: common::Time GetTime() const;
    public: double GetDepth() const;
    public: double GetPressure() const;
    public: double GetLatitude() const;

    protected: virtual bool UpdateImpl(const bool _force);

    protected:
        physics::LinkPtr parentLink;
        mutable std::mutex mutex;
    
        common::SphericalCoordinatesPtr sphericalCoordinates;
        transport::PublisherPtr depthPub;
        std::string topicName;
        ds_sim::msgs::PressureDepth msg;
  }; // class declaration
}; // namespace sensors
}; // namespace gazebo

// created with the GZ_REGISTER_STATIC_SENSOR macro
extern void RegisterDsrosDepthSensor();

