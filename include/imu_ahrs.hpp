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

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>

#include <slerp.hpp>

#include "mahony/MahonyAHRS.h"

namespace imu_ahrs
{

  class ImuAhrs : public rclcpp::Node
  {
  public:
    ImuAhrs();

  private:
    void setup();

    void accelCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void gyroCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
    void updateFilter();
    // void publishTF();
    void pub_data();

  private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_sub_, gyro_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    mahony::Mahony mahony_;
    Slerp slerp_;

    sensor_msgs::msg::Imu imu_msg;

    double ax = 0, ay = 0, az = 0;
    double gx = 0, gy = 0, gz = 0;
    double mx = 0, my = 0, mz = 0;
    
    bool yaw_initialized_ = false;
    double initial_yaw_ = 0.0;
  };

} // namespace imu_ahrs
