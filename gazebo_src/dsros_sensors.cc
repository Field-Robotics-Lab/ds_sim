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

#include "dsros_depth.hh"
#include "dsros_ins.hh"
#include "dsros_dvl.hh"

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
        RegisterDsrosInsSensor();
        RegisterDsrosDvlSensor();

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

