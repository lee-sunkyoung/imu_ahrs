#include <functional>
#include <imu_ahrs.hpp>

namespace imu_ahrs
{

  ImuAhrs::ImuAhrs() : Node("imu_ahrs")
  {
    this->setup();
    // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  void ImuAhrs::setup()
  {
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
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                      std::bind(&ImuAhrs::pub_data, this));
    mahony_.begin(10); // 10 Hz
  }
  void ImuAhrs::accelCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    ax = msg->vector.x - 0.23;
    ay = msg->vector.y - 0.25;
    az = msg->vector.z;
  }

  void ImuAhrs::gyroCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    gx = msg->vector.x; // deg/s로 들어옴
    gy = msg->vector.y;
    gz = msg->vector.z;
  }

  void ImuAhrs::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    mx = msg->magnetic_field.x;
    my = msg->magnetic_field.y;
    mz = msg->magnetic_field.z;
  }
  void ImuAhrs::updateFilter()
  {
    mahony_.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    slerp_.calc(mahony_.q0, mahony_.q1, mahony_.q2, mahony_.q3);

    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = static_cast<float>((std::abs(gx) < 0.1) ? 0.0 : gx);
    imu_msg.angular_velocity.y = static_cast<float>((std::abs(gy) < 0.1) ? 0.0 : gy);
    imu_msg.angular_velocity.z = static_cast<float>((std::abs(gz) < 0.1) ? 0.0 : gz);

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.orientation.w = slerp_.Qw;
    imu_msg.orientation.x = slerp_.Qx;
    imu_msg.orientation.y = slerp_.Qy;
    imu_msg.orientation.z = slerp_.Qz;
    imu_msg.orientation_covariance[0] = 0.0005; // roll
    imu_msg.orientation_covariance[4] = 0.0005; // pitch
    imu_msg.orientation_covariance[8] = 0.001;  // yaw (slightly more uncertain)
    imu_msg.angular_velocity_covariance[0] = 0.0001;
    imu_msg.angular_velocity_covariance[4] = 0.0001;
    imu_msg.angular_velocity_covariance[8] = 0.0001;
    imu_msg.linear_acceleration_covariance[0] = 0.0005;
    imu_msg.linear_acceleration_covariance[4] = 0.0005;
    imu_msg.linear_acceleration_covariance[8] = 0.0008;
    // std::cout<<slerp_.roll<<" , "<<slerp_.pitch<<" , "<<slerp_.yaw<<" , "<<std::endl;

    tf2::Quaternion q(
        slerp_.Qx,
        slerp_.Qy,
        slerp_.Qz,
        slerp_.Qw);

    // 쿼터니언을 roll, pitch, yaw로 변환
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (!yaw_initialized_)
    {
      initial_yaw_ = yaw; // 첫 yaw 저장
      yaw_initialized_ = true;
    }

    // yaw를 초기 yaw에 대해 상대값으로 만들기 (즉, 시작시 yaw=0)
    yaw -= initial_yaw_;

    tf2::Quaternion q_fixed;
    q_fixed.setRPY(roll, pitch, yaw);
    q_fixed.normalize();

    imu_msg.orientation = tf2::toMsg(q_fixed);

    // imu_msg.orientation 쿼터니언을 tf2::Quaternion으로 변환
    sensor_msgs::msg::Imu buf_msg;
    buf_msg = imu_msg;
    tf2::Quaternion orientation;
    tf2::fromMsg(buf_msg.orientation, orientation);

    // 중력 벡터 (지구 중력 가속도)
    const double g = 9.81;

    // 쿼터니언을 이용해 중력 벡터를 IMU 좌표계로 회전시킴
    // 중력 벡터는 지구 기준 (0, 0, g)
    tf2::Vector3 gravity_earth(0, 0, g);

    // 중력 벡터를 IMU 프레임으로 변환 (역방향 회전)
    tf2::Vector3 gravity_imu = tf2::quatRotate(orientation, gravity_earth);

    // 원래 가속도에서 중력 성분 제거
    double acc_x = ax - gravity_imu.x();
    double acc_y = ay - gravity_imu.y();
    double acc_z = az - gravity_imu.z();

    // 보정된 가속도 다시 imu_msg에 저장
    imu_msg.linear_acceleration.x = 0.0;//static_cast<float>((std::abs(acc_x) < 0.5) ? 0.0 : acc_x);
    imu_msg.linear_acceleration.y = 0.0;//static_cast<float>((std::abs(acc_y) < 0.5) ? 0.0 : acc_y);
    imu_msg.linear_acceleration.z = 0.0;//static_cast<float>((std::abs(acc_z) < 0.5) ? 0.0 : acc_z);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "velodyne";
    pose_msg.pose.orientation = imu_msg.orientation;

    pose_pub_->publish(pose_msg);
    // double roll, pitch, yaw;
    // tf2::Quaternion orientation;
    // tf2::fromMsg(imu_msg.orientation, orientation);
    // tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    // float acc_x = static_cast<float>(imu_msg.linear_acceleration.x) + sin(pitch) * 9.81;
    // float acc_y = static_cast<float>(imu_msg.linear_acceleration.y) - cos(pitch) * sin(roll) * 9.81;
    // float acc_z = static_cast<float>(imu_msg.linear_acceleration.z) - cos(pitch) * cos(roll) * 9.81;

    // imu_msg.linear_acceleration.x = acc_x;
    // imu_msg.linear_acceleration.y = acc_y;
    // // imu_msg.linear_acceleration.z = acc_z;

    // RCLCPP_INFO(this->get_logger(), "Quat: w=%.4f, x=%.4f, y=%.4f, z=%.4f",
    // orientation.w(), orientation.x(), orientation.y(), orientation.z());

    // publishTF();
  }
  void ImuAhrs::pub_data()
  {
    imu_pub_->publish(imu_msg);
  }
  // void ImuAhrs::publishTF()
  // {
  //   geometry_msgs::msg::TransformStamped t;
  //   t.header.stamp = this->get_clock() > now();
  //   t.header.frame_id = "map";
  //   t.child_frame_id = "odom";

  //   // Set translation & rotation here
  //   t.transform.translation.x = 0.0;
  //   t.transform.translation.y = 0.0;
  //   t.transform.translation.z = 0.0;

  //   t.transform.rotation.x = slerp_.Qx;
  //   t.transform.rotation.y = slerp_.Qy;
  //   t.transform.rotation.z = slerp_.Qz;
  //   t.transform.rotation.w = slerp_.Qw;
  //   tf_broadcaster_->sendTransform(t);
  // }
} // namespace imu_ahrs

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_ahrs::ImuAhrs>());
  rclcpp::shutdown();

  return 0;
}