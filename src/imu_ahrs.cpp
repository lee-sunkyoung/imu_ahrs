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
    // 1Hz로 드리프트 없는 상태에서 평균 찍기
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "gyro (raw): %.5f %.5f %.5f [units?]", gx, gy, gz);

    // ====== 1) 상태/게이트 판단 ======
    // 정지 판정: 자이로(각속도) + 가속도 크기(g) 근처를 함께 체크
    constexpr double G = 9.81;
    constexpr double STILL_GYR_THR = 0.0087; // rad/s ≈ 0.5 deg/s
    constexpr double STILL_ACC_THR = 0.15;   // m/s^2, |a|-g 허용 오차

    const bool still_gyr = (std::abs(gx) + std::abs(gy) + std::abs(gz)) < STILL_GYR_THR;
    const double anorm = std::sqrt(ax * ax + ay * ay + az * az);
    const bool still_acc = std::abs(anorm - G) < STILL_ACC_THR;
    const bool still = still_gyr && still_acc;

    // 자기장 게이팅: MagneticField.msg 단위는 Tesla
    // 지구 자기장 크기: 대략 25~65 μT 범위
    float mmx = static_cast<float>(mx);
    float mmy = static_cast<float>(my);
    float mmz = static_cast<float>(mz);
    const float B_MIN = 25e-6f, B_MAX = 65e-6f;
    const float bnorm = std::sqrt(mmx * mmx + mmy * mmy + mmz * mmz);
    bool mag_ok = (bnorm > B_MIN && bnorm < B_MAX);
    if (!mag_ok)
    {
      mmx = mmy = mmz = 0.0f;
    } // 비정상일 땐 IMU 모드로 (Mahony가 자동 처리)

    // ====== 2) Mahony 입력 준비 (deg/s로) ======
    // Mahony 내부에서 deg->rad, 입력은 deg/s
    //constexpr double RAD2DEG = 180.0 / M_PI;
    float gx_deg = static_cast<float>(gx); // * RAD2DEG
    float gy_deg = static_cast<float>(gy); // * RAD2DEG
    float gz_deg = static_cast<float>(gz); // * RAD2DEG

    // ZARU: 정지 상태일 때는 각속도 적분을 막아 드리프트 억제 (yaw 주로, 원하면 x,y도 0)
    if (still)
    {
      // yaw 드리프트 억제 목적이면 gz만 0으로 둬도 충분. 필요시 아래 두 줄도 0으로.
      gx_deg = 0.0f; gy_deg = 0.0f;
      gz_deg = 0.0f;
    }

    // ====== 3) 필터 업데이트 ======
    mahony_.update(gx_deg, gy_deg, gz_deg,
                   static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az),
                   mmx, mmy, mmz);
    //slerp_.calc(mahony_.q0, mahony_.q1, mahony_.q2, mahony_.q3);
    // ====== 4) 메시지 채우기 ======
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    // ROS 규격은 rad/s → 드라이버 원본(rad/s)을 그대로 사용 (퍼블리시 값은 보정하지 않음)
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

    // ====== 5) yaw 초기 기준(0) 맞추기 ======
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

    // 쿼터니언 부호 고정(시각적 튐 방지)
    if (q_out.w() < 0)
    {
      q_out = tf2::Quaternion(-q_out.x(), -q_out.y(), -q_out.z(), -q_out.w());
    }

    imu_msg.orientation = tf2::toMsg(q_out);

    // 맵핑 옵션
    if (!use_linear_acc)
    {
      imu_msg.linear_acceleration.x = 0.0;
      imu_msg.linear_acceleration.y = 0.0;
      imu_msg.linear_acceleration.z = 0.0;
    }

    // ====== 6) 디버깅: 헤딩 1Hz + 자기 OK 여부/크기 출력 ======
    {
      double rr, pp, yy;
      tf2::Matrix3x3(q_out).getRPY(rr, pp, yy);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Heading(ENU) %.2f deg (%.3f rad) | still=%d | mag_ok=%d | |B|=%.1f uT",
                           yy * 180.0 / M_PI, yy, (int)still, (int)mag_ok, bnorm * 1e6f);
    }

    // ====== 7) Pose 퍼블리시 (시각화 프레임 주의) ======
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "velodyne"; // RViz TF가 늦게 뜨면 일시적으로 "imu_link"로 보며 확인 권장
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