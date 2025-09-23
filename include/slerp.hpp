//선형보간
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#define M_PI 3.1415926535

// class Slerp { 
//  private:
//   Eigen::Quaterniond q_prev_ = Eigen::Quaterniond::Identity();
//   bool initialized_ = false;

//   // SLERP 비율 (0.0~1.0) 작을수록 더 부드러우나 지연
//   const double alpha_ = 0.05; 
//   //센서 샘플링 주기 (100 Hz 0.05)
  
//  public:
//   Slerp() {}

//   double Qw = 0, Qx = 0, Qy = 0, Qz = 0;
//   double roll = 0, pitch = 0, yaw = 0;
//   void calc(double qw, double qx, double qy, double qz) {
//     Eigen::Quaterniond q_in(qw, qx, qy, qz);

//     Eigen::Quaterniond q_filtered;

//     // slerp
//     if (!initialized_) {
//       q_prev_ = q_in;
//       initialized_ = true;
//       q_filtered = q_in;
//     } else {
//       q_filtered = q_prev_.slerp(alpha_, q_in);
//       q_filtered.normalize();
//       q_prev_ = q_filtered;
//     }

//     Qw = q_filtered.w();
//     Qx = q_filtered.x();
//     Qy = q_filtered.y();
//     Qz = q_filtered.z();
//   }
// };

class Slerp {
 private:
  Eigen::Quaterniond q_prev_ = Eigen::Quaterniond::Identity();
  bool initialized_ = false;

  // 100 Hz에서 alpha=0.05면 대략 τ≈(1/alpha)*dt ≈ 0.2s 정도의 1차 저역 통과 느낌
  const double alpha_ = 0.05;

 public:
  double Qw = 1, Qx = 0, Qy = 0, Qz = 0;

  void calc(double qw, double qx, double qy, double qz) {
    // Eigen 순서: (w,x,y,z)
    Eigen::Quaterniond q_in(qw, qx, qy, qz);
    q_in.normalize();

    if (!initialized_) {
      q_prev_ = q_in;
      initialized_ = true;
    } else {
      // 최단 경로 강제: dot<0면 q_in 부호 반전
      double dot = q_prev_.w()*q_in.w() + q_prev_.x()*q_in.x()
                 + q_prev_.y()*q_in.y() + q_prev_.z()*q_in.z();
      if (dot < 0.0) q_in.coeffs() *= -1.0;  // coeffs() = (x,y,z,w)

      q_prev_ = q_prev_.slerp(alpha_, q_in);
      q_prev_.normalize();
    }

    Qw = q_prev_.w();
    Qx = q_prev_.x();
    Qy = q_prev_.y();
    Qz = q_prev_.z();
  }
};