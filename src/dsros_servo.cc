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
#include <thread>

#include <boost/bind.hpp>

namespace gazebo {
class DsrosServo : public ModelPlugin {
  public:

    virtual ~DsrosServo() {
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

        // _sdf->SetRequired does seem to do anything...
        if (!(_sdf->HasElement("joint_name") 
            &&_sdf->HasElement("cmd_topic")
            &&_sdf->HasElement("state_topic"))) {
            ROS_FATAL_STREAM("ROS plugin " <<_sdf->GetAttribute("name")
            <<" for Gazebo is missing one or more tags (you need 'joint_name', 'cmd_topic', and 'state_topic'");
            return;
        }

        // get the namespace
        if (_sdf->HasElement("robotNamespace")) {
            robotNamespace = _sdf->Get<std::string>("robotNamespace");
        } else {
            robotNamespace = _parent->GetName();
        }

        // parse some stuff from the SDF file
        joint_name = _sdf->Get<std::string>("joint_name");
        cmd_topic_name = _sdf->Get<std::string>("cmd_topic");
        state_topic_name = _sdf->Get<std::string>("state_topic");
        
        // Remove any double-quotes
        joint_name.erase(std::remove(joint_name.begin(), joint_name.end(), '\"'),joint_name.end());
        cmd_topic_name.erase(std::remove(cmd_topic_name.begin(), cmd_topic_name.end(), '\"'),cmd_topic_name.end());
        state_topic_name.erase(std::remove(state_topic_name.begin(), state_topic_name.end(), '\"'),state_topic_name.end());

        // Acquire the right joint
        ROS_DEBUG_STREAM("DsrosServo: Grabbing joint " <<joint_name <<"...");
        joint = _parent->GetJoint(joint_name);
        if (!joint) {
            ROS_FATAL_STREAM("ROS plugin " <<_sdf->GetAttribute("name")
            << " does not name a valid joint! Bailing on plugin...");
            return;
        }
        if (joint->GetAngleCount() > 1) {
            ROS_FATAL_STREAM("ROS/Gazebo plugin " <<_sdf->GetAttribute("name")
            << " may not specify a revolute joint!  This can cause undefined behavior.  CONTINUING!");
        }

        // initialize our state variable
        state.stamp = ros::Time().now();
        state.servo_name = joint_name;

        // Initialize connection to ROS...
        ROS_DEBUG_STREAM("DsrosServo: Initializing internal ROS node...");
        rosNode.reset(new ros::NodeHandle(robotNamespace));

        // Subscribe to a command topic
        ROS_DEBUG_STREAM("DsrosServo: Subscribing to command topic \"" 
                <<cmd_topic_name <<"\"...");
        rosSub = rosNode->subscribe<ds_msgs::ServoCmd>(cmd_topic_name, 1, boost::bind(&DsrosServo::SetCommandedAngle, this, _1));

        // publish state
        ROS_DEBUG_STREAM("DsrosServo: Publishing on state topic \"" 
                <<state_topic_name <<"\"...");
        rosPub = rosNode->advertise<ds_msgs::ServoState>(state_topic_name, 10);

        // prepare some reference times
        double updateRate = 5;
        if (_sdf->HasElement("updateRate")) {
            updateRate = _sdf->Get<double>("updateRate");
        }
        rosPublishPeriod = gazebo::common::Time(1.0/updateRate);
        lastRosPublished = gazebo::common::Time(0.0);


        // Listen to the update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DsrosServo::OnUpdate, this, _1));
    }


    void OnUpdate(const common::UpdateInfo& _info) {
        // do the kinematics
        state.actual_radians = state.cmd_radians;
        
        // update the joint angle
        joint->SetPosition(0,state.actual_radians);
        joint->SetVelocity(0,0);
        joint->SetForce(0,0);

        ROS_DEBUG_STREAM("Dsros: Setting servo to angle=" <<state.actual_radians*180/M_PI <<" deg");

        // publish a state message
        if (_info.simTime > lastRosPublished + rosPublishPeriod) {
            state.stamp = ros::Time().now();
            rosPub.publish(state);
            lastRosPublished = _info.simTime;
        }
    }

    void SetCommandedAngle(const ds_msgs::ServoCmd::ConstPtr &_msg) {
        state.cmd_radians = _msg->cmd_radians;
        ROS_DEBUG_STREAM("Dsros: Commanded to " <<state.cmd_radians*180/M_PI <<" deg");
    }

  protected:
    physics::JointPtr joint;
    event::ConnectionPtr updateConnection;

    gazebo::common::Time rosPublishPeriod, lastRosPublished;
    std::string robotNamespace;
    std::string joint_name, cmd_topic_name, state_topic_name;

    // we'll use a thruster state internally to store the thruster state
    ds_msgs::ServoState state;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    ros::Publisher rosPub;

};

GZ_REGISTER_MODEL_PLUGIN(DsrosServo)
};

