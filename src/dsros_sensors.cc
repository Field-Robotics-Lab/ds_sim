/*
 *
 * dsros_sensors.cc
 *
 * Ian Vaughn, 2017 Nov 16
 *
 *
 * Ok, this is kinda crazy, so bear with me.  
 *
 * We want to add custom sensors to gazebo.  Gazebo makes it
 * easy to do this because it was brilliantly architected-- but
 * only if you recompile gazebo (grr)
 * 
 * The reason is that you need to register the new sensors
 * with gazebo::sensors::SensorFactory AT RUNTIME.
 * All that makes sense, but how
 * can we do that without recompiling gazebo?
 *
 * Fortunately, there is a way! Use a System plugin
 * to load the new sensors at startup!  As an added bonus, all the object
 * code required to add the sensor now lives in this common
 * sensors.so object rather than a custom plugin object.
 *
 * This is a very-nearly-epic hack
 */

#include <gazebo/gazebo.hh>

#include "dsros_depthsensor.cc"

namespace gazebo {
class RegisterDsRosSensorsPlugin : public SystemPlugin {

    //typedef Sensor* (*DsrosSensorFactoryFn) ();

    //////////////////////////////////////////////
    // \brief Destructor
    public: virtual ~RegisterDsRosSensorsPlugin() {

    }


    //////////////////////////////////////////////
    // \brief Called after the plugin has been constructed
    public: void Load(int _argc, char** _argv) {
        gzdbg <<"Loading DS ROS Sensors! (for real!)" <<std::endl;
        RegisterDsrosDepthSensor();

        std::vector<std::string> types;
        gazebo::sensors::SensorFactory::GetSensorTypes(types);

        for (const std::string& t : types) {
            gzdbg <<"Sensor type: \"" <<t <<"\"" <<std::endl;
        }
    }

/*
    public: static void RegisterSensor(const std::string & _className,
                                       DsrosSensorFactoryFn _factoryfn) {

    }
    */
    
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(RegisterDsRosSensorsPlugin);

};

