#include <functional>
#include <imu_ahrs.hpp>

namespace imu_ahrs
{

  ImuAhrs::ImuAhrs() : Node("imu_ahrs")
  {
    this->declare_parameter<bool>("use_linear_acc", false);
    this->setup();
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
    this->get_parameter("use_linear_acc", use_linear_acc);
    RCLCPP_INFO(this->get_logger(), "use_linear_acc: %s",
                use_linear_acc ? "true" : "false");

    mahony_.begin(100); // 10 Hz
  }
  void ImuAhrs::accelCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    ax = msg->vector.x;
    ay = msg->vector.y;
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
    // 정지 판정 ang_v + acc
    constexpr double G = 9.81;
    constexpr double STILL_GYR_THR = 0.0087; // rad/s ≈ 0.5 deg/s
    constexpr double STILL_ACC_THR = 0.15;   // m/s^2, |a|-g 허용 오차

    const bool still_gyr = (std::abs(gx) + std::abs(gy) + std::abs(gz)) < STILL_GYR_THR;
    const double anorm = std::sqrt(ax * ax + ay * ay + az * az);
    const bool still_acc = std::abs(anorm - G) < STILL_ACC_THR;
    const bool still = still_gyr && still_acc;

    float mmx = static_cast<float>(mx);
    float mmy = static_cast<float>(my);
    float mmz = static_cast<float>(mz);
    const float B_MIN = 25e-6f, B_MAX = 65e-6f; //지구자기장 25~65 μT
    const float bnorm = std::sqrt(mmx * mmx + mmy * mmy + mmz * mmz);
    bool mag_ok = (bnorm > B_MIN && bnorm < B_MAX);
    if (!mag_ok)
    {
      mmx = mmy = mmz = 0.0f;
    } // 비정상일 땐 IMU

    // Mahony 내부에서 deg->rad
    float gx_deg = static_cast<float>(gx); // * RAD2DEG
    float gy_deg = static_cast<float>(gy); // * RAD2DEG
    float gz_deg = static_cast<float>(gz); // * RAD2DEG

    // 정지일 때 각속도 적분x
    if (still)
    {
      gx_deg = 0.0f; gy_deg = 0.0f;
      gz_deg = 0.0f;
    }

    mahony_.update(gx_deg, gy_deg, gz_deg,
                   static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az),
                   mmx, mmy, mmz);
    //slerp_.calc(mahony_.q0, mahony_.q1, mahony_.q2, mahony_.q3);

    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    //rad/s
    imu_msg.angular_velocity.x = std::abs(gx) < 1e-3 ? 0.0 : gx;
    imu_msg.angular_velocity.y = std::abs(gy) < 1e-3 ? 0.0 : gy;
    imu_msg.angular_velocity.z = std::abs(gz) < 1e-3 ? 0.0 : gz;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.orientation_covariance[0] = 0.0005; // roll
    imu_msg.orientation_covariance[4] = 0.0005; // pitch
    imu_msg.orientation_covariance[8] = 0.001;  // yaw
    imu_msg.angular_velocity_covariance[0] = 0.0001;
    imu_msg.angular_velocity_covariance[4] = 0.0001;
    imu_msg.angular_velocity_covariance[8] = 0.0001;
    imu_msg.linear_acceleration_covariance[0] = 0.0005;
    imu_msg.linear_acceleration_covariance[4] = 0.0005;
    imu_msg.linear_acceleration_covariance[8] = 0.0008;

    tf2::Quaternion qout(mahony_.q1, mahony_.q2, mahony_.q3, mahony_.q0);
    if (!yaw_initialized_ && warming_up++ >= 300)
    {
      double r, p, y;
      tf2::Matrix3x3(qout).getRPY(r, p, y);
      initial_yaw_ = y;
      yaw_initialized_ = true;
    }
    tf2::Quaternion q_out = qout;
    if (yaw_initialized_)
    {
      tf2::Quaternion q_off;
      q_off.setRPY(0, 0, -initial_yaw_);
      q_out = q_off * qout;
      q_out.normalize();
    }

    if (q_out.w() < 0)
    {
      q_out = tf2::Quaternion(-q_out.x(), -q_out.y(), -q_out.z(), -q_out.w());
    }

    imu_msg.orientation = tf2::toMsg(q_out);

    if (!use_linear_acc)
    {
      imu_msg.linear_acceleration.x = 0.0;
      imu_msg.linear_acceleration.y = 0.0;
      imu_msg.linear_acceleration.z = 0.0;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "velodyne";
    pose_msg.pose.orientation = imu_msg.orientation;
    pose_pub_->publish(pose_msg);
  }

  void ImuAhrs::pub_data()
  {
    imu_pub_->publish(imu_msg);
  }
} // namespace imu_ahrs

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_ahrs::ImuAhrs>());
  rclcpp::shutdown();

  return 0;
}