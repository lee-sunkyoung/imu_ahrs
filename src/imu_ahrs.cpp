#include <functional>
#include <imu_ahrs.hpp>

namespace imu_ahrs {

ImuAhrs::ImuAhrs() : Node("imu_ahrs") { this->setup(); }

void ImuAhrs::setup() {
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_data", 10);
  accel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/acceleration", 10,
      std::bind(&ImuAhrs::accelCallback, this, std::placeholders::_1));

  gyro_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/angular_velocity", 10,
      std::bind(&ImuAhrs::gyroCallback, this, std::placeholders::_1));

  mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/imu/mag", 10,
      std::bind(&ImuAhrs::magCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ImuAhrs::updateFilter, this));

  mahony_.begin(100);  // 100 Hz
}
void ImuAhrs::accelCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  ax = msg->vector.x;
  ay = msg->vector.y;
  az = msg->vector.z;
}

void ImuAhrs::gyroCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  gx = msg->vector.x;  // deg/s로 들어옴
  gy = msg->vector.y;
  gz = msg->vector.z;
}

void ImuAhrs::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
  mx = msg->magnetic_field.x;
  my = msg->magnetic_field.y;
  mz = msg->magnetic_field.z;
}
void ImuAhrs::updateFilter() {
  mahony_.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  slerp_.calc(mahony_.q0,mahony_.q1,mahony_.q2,mahony_.q3);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = this->get_clock()->now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  imu_msg.orientation.w = slerp_.Qw;
  imu_msg.orientation.x = slerp_.Qx;
  imu_msg.orientation.y = slerp_.Qy;
  imu_msg.orientation.z = slerp_.Qz;

  //std::cout<<slerp_.roll<<" , "<<slerp_.pitch<<" , "<<slerp_.yaw<<" , "<<std::endl;

  imu_pub_->publish(imu_msg);
  
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->get_clock()->now();
  pose_msg.header.frame_id = "pose";
  pose_msg.pose.orientation = imu_msg.orientation;

  pose_pub_->publish(pose_msg);
}
}  // namespace imu_ahrs

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_ahrs::ImuAhrs>());
  rclcpp::shutdown();

  return 0;
}