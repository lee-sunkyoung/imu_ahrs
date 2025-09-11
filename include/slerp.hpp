//선형보간
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#define M_PI 3.1415926535

class Slerp {
 private:
  Eigen::Quaterniond q_prev_ = Eigen::Quaterniond::Identity();
  bool initialized_ = false;

  // SLERP 비율 (0.0~1.0) 작을수록 더 부드러우나 지연
  const double alpha_ = 0.05; 
  //센서 샘플링 주기 (100 Hz 0.05)
  
 public:
  Slerp() {}

  double Qw = 0, Qx = 0, Qy = 0, Qz = 0;
  double roll = 0, pitch = 0, yaw = 0;
  void calc(double qw, double qx, double qy, double qz) {
    Eigen::Quaterniond q_in(qw, qx, qy, qz);

    Eigen::Quaterniond q_filtered;

    // slerp
    if (!initialized_) {
      q_prev_ = q_in;
      initialized_ = true;
      q_filtered = q_in;
    } else {
      q_filtered = q_prev_.slerp(alpha_, q_in);
      q_filtered.normalize();
      q_prev_ = q_filtered;
    }

    Qw = q_filtered.w();
    Qx = q_filtered.x();
    Qy = q_filtered.y();
    Qz = q_filtered.z();
  }
};