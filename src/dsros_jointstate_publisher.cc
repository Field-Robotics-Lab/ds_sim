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
//
// Ian Vaughn, October 2017
// 
// Based on the simplified 4-DOF dynamics model
// in ROV

//#include <sentry_sim/SentryDynamics_plugin.hh>

// REV: MELODIC 1 0

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#if GAZEBO_MAJOR_VERSION > 7
#include <gazebo/common/CommonIface.hh>
#else
#include <gazebo/common/common.hh>
#endif

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <thread>

#include <boost/bind.hpp>

namespace gazebo {

class DsJointStatePublisher : public ModelPlugin {
  public:
    virtual ~DsJointStatePublisher() {
       if (updateConnection) {
#if GAZEBO_MAJOR_VERSION > 7
      // Melodic API change
      // Deprecation: public: void Events::Disconnect.*(ConnectionPtr);
      // Replacement: Delete the Connection object, perhaps by calling reset() on its smart pointer.
      this->updateConnection.reset();
#else
      gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
        }

        if (rosNode) {
            rosNode->shutdown();
        }
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        // Check on ROS
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        // Get the proper robot name
        this->model = _parent;
        if (_sdf->HasElement("robotNamespace")) {
            this->robotNamespace = _sdf->Get<std::string>("robotNamespace");
        } else {
            this->robotNamespace = this->model->GetName();
        }

        // Connect to the ROS node
        ROS_INFO_STREAM("ds_jointstatepublisher: Initializing internal ROS node...");
        rosNode.reset(new ros::NodeHandle(robotNamespace));

        sign = 1.0;
        if (_sdf->HasElement("invertSign")) {
            if (_sdf->Get<bool>("invertSign")) {
                sign = -1.0;
            }
        }

        // check to see if we're disabled by a rosparam
        disable = false;
        if (_sdf->HasElement("disableParam")) {
            std::string disableParam = _sdf->Get<std::string>("disableParam");

            if (rosNode->hasParam(disableParam) &&
               rosNode->getParam(disableParam, disable)) {
               ROS_INFO_STREAM("Loaded sim_jointstate_pub::disable=" <<disable);
            } else {
               ROS_ERROR_STREAM("Unable to load sim_jointstate_pub::disable from " 
                               <<rosNode->resolveName(disableParam));
               disable = false;
            }

            if (disable) {
                ROS_ERROR_STREAM("jointstate publisher disabling itself!");
            }
        }


        // prepare some reference times
        double updateRate = 10;
        if (_sdf->HasElement("updateRate")) {
            updateRate = _sdf->Get<double>("updateRate");
        }
        rosPublishPeriod = gazebo::common::Time(1.0/updateRate);
        lastRosPublished = gazebo::common::Time(0.0);

        // Setup our publisher
        ROS_INFO_STREAM("ds_jointstatepublisher: Advertising topic to publish on...");
        if (!disable) {
            rosPub = rosNode->advertise<sensor_msgs::JointState>("joint_states", 10);
            // Listen to the update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&DsJointStatePublisher::OnUpdate, this, _1));
        }
    }

    void OnUpdate(const common::UpdateInfo& _info) {
        if (_info.simTime < lastRosPublished + rosPublishPeriod) {
            return;
        }
        if (disable) {
            lastRosPublished += rosPublishPeriod;
            return;
        }

        sensor_msgs::JointState jointState;
        jointState.header.stamp = ros::Time::now();
        jointState.name.resize(this->model->GetJointCount());
        jointState.position.resize(this->model->GetJointCount());
        jointState.velocity.resize(this->model->GetJointCount());
        jointState.effort.resize(this->model->GetJointCount());

        int i=0;
        for (const physics::JointPtr& joint : this->model->GetJoints()) {
            jointState.name[i] = joint->GetName();

            // check if the joint should be ignored
            if (joint->HasType(gazebo::physics::Base::EntityType::FIXED_JOINT)
#if GAZEBO_MAJOR_VERSION > 7
                || joint->DOF() > 1
                || joint->LowerLimit(0) == 0 && joint->UpperLimit(0) == 0) {
#else
                || joint->GetAngleCount() > 1
                || joint->GetLowerLimit(0).Radian() == 0 && joint->GetUpperLimit(0).Radian() == 0) {
#endif
                // ignore this joint
                jointState.position[i] = 0;
                jointState.velocity[i] = 0;
                jointState.effort[i] = 0;
            } else {
#if GAZEBO_MAJOR_VERSION > 7
                jointState.position[i] = sign*joint->Position(0);
#else
                jointState.position[i] = sign*joint->GetAngle(0).Radian();
#endif
                jointState.velocity[i] = joint->GetVelocity(0);
                jointState.effort[i] = joint->GetForce(0);
            }
            i++;
        }
        rosPub.publish(jointState);
        lastRosPublished += rosPublishPeriod;
    }

  protected:
    event::ConnectionPtr updateConnection;
    std::string robotNamespace;
    physics::ModelPtr model;
    double sign;
    bool disable;

    gazebo::common::Time rosPublishPeriod, lastRosPublished;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher rosPub;

};

GZ_REGISTER_MODEL_PLUGIN(DsJointStatePublisher);


}; // namespace gazebo

