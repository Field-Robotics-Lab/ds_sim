//
// Ian Vaughn, October 2017
// 
// Based on the simplified 4-DOF dynamics model
// in ROV

//#include <sentry_sim/SentryDynamics_plugin.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ds_msgs/ServoCmd.h>
#include <ds_msgs/ServoState.h>
#include <sensor_msgs/JointState.h>
#include <thread>

#include <boost/bind.hpp>

namespace gazebo {

class DsJointStatePublisher : public ModelPlugin {
  public:
    virtual ~DsJointStatePublisher() {
        if (updateConnection) {
            gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
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

        // prepare some reference times
        double updateRate = 10;
        if (_sdf->HasElement("updateRate")) {
            updateRate = _sdf->Get<double>("updateRate");
        }
        rosPublishPeriod = gazebo::common::Time(1.0/updateRate);
        lastRosPublished = gazebo::common::Time(0.0);

        // Setup our publisher
        ROS_INFO_STREAM("ds_jointstatepublisher: Advertising topic to publish on...");
        rosPub = rosNode->advertise<sensor_msgs::JointState>("joint_states", 10);
        // Listen to the update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DsJointStatePublisher::OnUpdate, this, _1));
    }

    void OnUpdate(const common::UpdateInfo& _info) {
        if (_info.simTime < lastRosPublished + rosPublishPeriod) {
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
                || joint->GetAngleCount() > 1
                || joint->GetLowerLimit(0).Radian() == 0 && joint->GetUpperLimit(0).Radian() == 0) {
                // ignore this joint
                jointState.position[i] = 0;
                jointState.velocity[i] = 0;
                jointState.effort[i] = 0;
            } else {
                jointState.position[i] = sign*joint->GetAngle(0).Radian();
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

    gazebo::common::Time rosPublishPeriod, lastRosPublished;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher rosPub;

};

GZ_REGISTER_MODEL_PLUGIN(DsJointStatePublisher);


}; // namespace gazebo

