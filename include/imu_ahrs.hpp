#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <slerp.hpp>

#include "mahony/MahonyAHRS.h"

namespace imu_ahrs {

class ImuAhrs : public rclcpp::Node {
 public:
  ImuAhrs();

 private:
  void setup();

  void accelCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void gyroCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
  void updateFilter();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_sub_, gyro_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mahony::Mahony mahony_;
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
  double mx = 0, my = 0, mz = 0;

  Slerp slerp_;

};

}  // namespace imu_ahrs
