

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ros/ros.h>
#include <string>
#include <ds_multibeam_msgs/MultibeamRaw.h>

namespace gazebo {

class dsrosRosResonSensor : public SensorPlugin {
 public:

  /// \brief Constructor
  dsrosRosResonSensor();

  /// \brief Destructor
  virtual ~dsrosRosResonSensor();

  /// \brief Load the sensor.
  /// \param sensor_ pointer to the sensor.
  /// \param sdf_ pointer to the sdf config file.
  virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

 protected:
  /// \brief Update the sensor
  virtual void UpdateChild(const gazebo::common::UpdateInfo &_info);

 private:

  double GaussianKernel(double mu, double sigma);
  bool LoadParameters();

  /// \brief ROS Node Handle
  ros::NodeHandle *node;

  /// \brief Reson publisher
  ros::Publisher reson_data_publisher;
  ros::Publisher pointcloud_publisher;

  ds_multibeam_msgs::MultibeamRaw mb_msg;
  // TODO: message (pointcloud_msg)

  /// \brief last time on which the data was published.
  common::Time last_time;

  /// \brief Pointer to the update event connection.
  gazebo::event::ConnectionPtr connection;

  /// \brief Pointer to the sensor.
  std::shared_ptr<sensors::RaySensor> sensor;

  /// \brief Pointer to the sdf config file.
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string frame_name;

  unsigned int seed;
  double update_rate;
  double gaussian_noise;

  // actual core data
  // TODO
};

}