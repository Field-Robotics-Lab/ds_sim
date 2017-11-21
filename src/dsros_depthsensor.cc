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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>

#include <mutex>

namespace gazebo {
namespace sensors {

class DsrosDepthSensor : public Sensor {
    public: DsrosDepthSensor() : Sensor(sensors::OTHER){
        gzdbg <<"Creating DsrosDepthSensor" <<std::endl;
    };

    public: virtual ~DsrosDepthSensor() {};

    public: virtual void Load(const std::string &_worldName,
                             sdf::ElementPtr _sdf) {
        Sensor::Load(_worldName, _sdf);
        gzdbg <<"Sensor " <<this->Name() <<" loading!" <<std::endl;
    }

    public: virtual void Load(const std::string &_worldName) {
        Sensor::Load(_worldName);

        physics::EntityPtr parentEntity = this->world->GetEntity(
                                                this->ParentName());
        this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);
        if (this->parentLink) {
            gzdbg <<"Sensor " <<this->Name() <<" loaded link " <<this->parentLink->GetName() <<std::endl;
        } else {
            gzdbg <<"Sensor " <<this->Name() <<" could not find parent link!" <<std::endl;
            if (parentEntity) {
                gzdbg <<"parent: " <<parentEntity->GetName() <<std::endl;
            }
        }

        gzdbg <<"Sensor Pose: " <<this->Pose() <<std::endl;
    }

    public: virtual void Init() {
        Sensor::Init();
    }

    protected: virtual bool UpdateImpl(const bool _force) {
        std::lock_guard<std::mutex> lock(this->mutex);

        // TODO

        return true;
    }

    public: virtual void Fini() {
        Sensor::Fini();
        parentLink.reset();
    }

    protected:
    physics::LinkPtr parentLink;
    mutable std::mutex mutex;

};

};

};

using namespace gazebo;
using namespace sensors;
GZ_REGISTER_STATIC_SENSOR("dsros_depth", DsrosDepthSensor);

