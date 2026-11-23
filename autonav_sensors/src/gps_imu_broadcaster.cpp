// C++ port (ROS 2 Jazzy) of gps_imu_broadcaster.py (Humble, Python)
// Same structure and function names as the Python version, to work with the current imu_gps_fastimu.ino
//
//  GPS,ms,lat,lon,alt,speed,hdop,sats,fix,heading,cardinal at 1Hz
//  IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz at 100Hz
// This is the "No microROS, only Serial" option
//
// Note that this is publishing various topics :
//    - standard ROS2:
//        - /imu/acc_gyro
//        - /imu/mag   (only if the publish_mag parameter is true)
//        - /gps/fix
//    - custom for compass, used in panorama:
//        - /heading
//        - /cardinal_compass (N, NE, E, etc)
//
// Run with fake data and no magnetometer topic:
//   ros2 run autonav_sensors gps_imu_broadcaster --ros-args -p simulated_data:=true -p publish_mag:=false

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Small helpers that Python gives you for free (strip, split, float)
static std::string strip(const std::string & s)
{
  size_t start = 0;
  size_t end = s.size();
  while (start < end && std::isspace(static_cast<unsigned char>(s[start]))) {
    start++;
  }
  while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    end--;
  }
  return s.substr(start, end - start);
}

static std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> parts;
  std::string item;
  std::stringstream ss(line);
  while (std::getline(ss, item, ',')) {
    parts.push_back(strip(item));
  }
  return parts;
}

// like Python's float(), throws if the text is not a number
static double to_float(const std::string & s)
{
  size_t used = 0;
  double value = 0.0;
  try {
    value = std::stod(s, &used);
  } catch (const std::exception &) {
    throw std::invalid_argument("not a number: '" + s + "'");
  }
  if (used != s.size()) {
    throw std::invalid_argument("not a number: '" + s + "'");
  }
  return value;
}

class IMUGPSNode : public rclcpp::Node
{
public:
  IMUGPSNode()
  : Node("imu_gps_node")
  {
    // Parameters (easy to override in launch files)
    this->declare_parameter<std::string>("port", "/dev/autonavESP32");
    this->declare_parameter<int>("baud", 115200);  // still matches with new arduino sketch
    this->declare_parameter<std::string>("frame_id", "gps_imu_link");  // match IMU, gps can be off by 5cm for all i care
    this->declare_parameter<bool>("simulated_data", false);
    this->declare_parameter<bool>("publish_mag", true);  // false = no /imu/mag topic at all

    // other internal parameters
    port_ = this->get_parameter("port").as_string();
    baud_ = static_cast<int>(this->get_parameter("baud").as_int());
    frame_id_ = this->get_parameter("frame_id").as_string();
    simulated_data_ = this->get_parameter("simulated_data").as_bool();
    publish_mag_ = this->get_parameter("publish_mag").as_bool();
    serial_fd_ = -1;

    // Publishers
    // GPS
    NavSatFix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    headingGPS_pub_ = this->create_publisher<std_msgs::msg::Float32>("/heading", 10);
    compassGPS_pub_ = this->create_publisher<std_msgs::msg::String>("/cardinal_compass", 10);  // N S E W

    // IMU
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/acc_gyro", 50);
    if (publish_mag_) {
      mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 50);
    }

    // Start Node
    init_serial_connection();  // Start Serial, or Simulated
    thread_ = std::thread(&IMUGPSNode::publish_data, this);  // Start all the other functions
    RCLCPP_INFO(this->get_logger(), "imu_gps_node: GPS IMU node started!");
  }

  ~IMUGPSNode()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  // ----------------------------------------------------------------------------------------------------
  // Serial Read Try function
  // ----------------------------------------------------------------------------------------------------
  void init_serial_connection()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "imu_gps_node: Trying to connect to ESP32 serial port %s at %d baud", port_.c_str(), baud_);

    speed_t speed;
    switch (baud_) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      default: speed = B0; break;
    }

    // open and set up the port, remember the error text if anything fails
    std::string error;
    int fd = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
      error = std::strerror(errno);
    } else if (speed == B0) {
      error = "unsupported baud rate";
      close(fd);
      fd = -1;
    } else {
      termios tty;
      std::memset(&tty, 0, sizeof(tty));
      tcgetattr(fd, &tty);
      cfmakeraw(&tty);
      cfsetispeed(&tty, speed);
      cfsetospeed(&tty, speed);
      tty.c_cflag |= (CLOCAL | CREAD);
      tty.c_cc[VMIN] = 0;    // read() gives up after VTIME if nothing arrives
      tty.c_cc[VTIME] = 10;  // 1.0 s, same as timeout=1.0 in the Python version
      if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        error = std::strerror(errno);
        close(fd);
        fd = -1;
      }
    }

    if (fd >= 0) {
      serial_fd_ = fd;
      RCLCPP_INFO(
        this->get_logger(), "imu_gps_node: Connected to %s at %d baud", port_.c_str(), baud_);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "imu_gps_node: Failed to connect to %s at %d baud: %s",
        port_.c_str(), baud_, error.c_str());
      if (simulated_data_) {
        RCLCPP_INFO(this->get_logger(), "imu_gps_node: Using simulated data fallback");
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "imu_gps_node: No ESP32 serial connection and no simulated data enabled");
      }
    }
  }  // END init_serial_connection()

  // like pyserial's readline(): returns false if nothing arrived within the 1 s timeout
  bool readline(std::string & line)
  {
    char c;
    while (true) {
      ssize_t n = read(serial_fd_, &c, 1);
      if (n < 0) {
        throw std::runtime_error(std::strerror(errno));
      }
      if (n == 0) {
        return false;
      }
      if (c == '\n') {
        line = serial_buffer_;
        serial_buffer_.clear();
        return true;
      }
      serial_buffer_ += c;
    }
  }

  // --------------------------------------------------
  // Top Level MSG Publisher - But going to Publish within handle_XXX() functions
  // --------------------------------------------------
  void publish_data()
  {
    int sim_count = 0;
    while (rclcpp::ok() && running_) {
      // serial is ok
      if (serial_fd_ >= 0) {
        std::string line;
        try {
          if (!readline(line)) {
            continue;  // timed out, nothing to read yet
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "imu_gps_node: Serial read error: %s", e.what());
          close(serial_fd_);
          serial_fd_ = -1;
          continue;
        }

        line = strip(line);
        if (line.empty()) {
          continue;
        }

        if (line.rfind("GPS,", 0) == 0) {
          handle_gps(line);
        } else if (line.rfind("IMU,", 0) == 0) {
          handle_imu(line);
        }
      }

      // no serial, but we can simulate
      else if (simulated_data_) {
        if (sim_count == 0) {
          RCLCPP_WARN(this->get_logger(), "imu_gps_node: No serial data, using simulated data");
        }
        if (sim_count % 10 == 0) {
          get_simulated_gps_data();  // 1 HZ
        }
        get_simulated_imu_data();  // 10 HZ, sleeps 0.1 inside
        sim_count++;
      }

      // no serial, no simulate
      else {
        RCLCPP_WARN(
          this->get_logger(),
          "imu_gps_node: No serial IMU data, no simulated data, restarting serial connections");
        std::this_thread::sleep_for(1s);  // not in the Python version, stops the log from flooding
        init_serial_connection();
      }
    }
  }

  // --------------------------------------------------
  // GPS handler for Simulated
  // --------------------------------------------------
  void get_simulated_gps_data()
  {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    msg.latitude = 48.8584;
    msg.longitude = 2.2945;
    msg.altitude = 32.0;

    msg.position_covariance = {
      65.7066, 0.0, 0.0,
      0.0, 65.7066, 0.0,
      0.0, 0.0, 25.0};  // (5 m)^2
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    std_msgs::msg::Float32 heading_msg;
    heading_msg.data = 1;
    std_msgs::msg::String compass_msg;
    compass_msg.data = "N";
    headingGPS_pub_->publish(heading_msg);
    compassGPS_pub_->publish(compass_msg);

    NavSatFix_pub_->publish(msg);
  }

  // ----------------------------------------------------------------------------------------------------
  // IMU handler for Simulated
  // ----------------------------------------------------------------------------------------------------
  void get_simulated_imu_data()
  {
    // flat face up on ground, X facing north
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;
    imu_msg.linear_acceleration.x = -0.0647;
    imu_msg.linear_acceleration.y = 0.0125;
    imu_msg.linear_acceleration.z = 9.8055;
    imu_msg.angular_velocity.x = 0.001;  // flat on ground
    imu_msg.angular_velocity.y = 0.001;
    imu_msg.angular_velocity.z = 0.001;
    imu_msg.orientation.w = 1.0;  // Orientation unknown
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation_covariance = {
      1.0e-2, 0.0, 0.0,
      0.0, 1.0e-2, 0.0,
      0.0, 0.0, 1.0e-2};
    imu_msg.linear_acceleration_covariance = {
      4.5359e-4, 0.0, 0.0,
      0.0, 4.5359e-4, 0.0,
      0.0, 0.0, 1.81434e-3};
    imu_msg.angular_velocity_covariance = {
      9.98194e-6, 0.0, 0.0,
      0.0, 6.59392e-6, 0.0,
      0.0, 0.0, 4.71762e-6};

    std::this_thread::sleep_for(100ms);  // should be 0.01 for 100HZ, but for simulation lets ease off for testing

    imu_pub_->publish(imu_msg);

    if (publish_mag_) {
      sensor_msgs::msg::MagneticField mag_msg;
      mag_msg.header.stamp = imu_msg.header.stamp;  // let's use same timestamp here
      mag_msg.header.frame_id = frame_id_;  // same frame ids for both mag and imu
      mag_msg.magnetic_field.x = 0.0000711;  // sample from log
      mag_msg.magnetic_field.y = 0.00013935;
      mag_msg.magnetic_field.z = 0.0001338;
      mag_msg.magnetic_field_covariance = {
        1.0e-2, 0.0, 0.0,
        0.0, 1.0e-2, 0.0,
        0.0, 0.0, 1.0e-2};
      mag_pub_->publish(mag_msg);
    }
  }

  // --------------------------------------------------
  // GPS handler for Serial
  //   Format: GPS,ms,lat,lon,alt,speed,hdop,sats,fix,heading,cardinal
  // --------------------------------------------------
  void handle_gps(const std::string & line)
  {
    try {
      std::vector<std::string> p = split(line);
      if (p.size() != 11) {
        throw std::invalid_argument("expected 11 fields, got " + std::to_string(p.size()));
      }
      // p = _, ms, lat, lon, alt, speed, hdop, sats, fix, heading, cardinal
      int fix = std::stoi(p[8]);

      sensor_msgs::msg::NavSatFix msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;

      if (fix) {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        msg.latitude = to_float(p[2]);
        msg.longitude = to_float(p[3]);
        msg.altitude = to_float(p[4]);

        msg.position_covariance = {
          65.7066, 0.0, 0.0,
          0.0, 65.7066, 0.0,
          0.0, 0.0, 25.0};  // (5 m)^2
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      } else {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      }

      NavSatFix_pub_->publish(msg);

      // Heading ========================================================================
      std_msgs::msg::Float32 heading_msg;
      heading_msg.data = static_cast<float>(to_float(p[9]));

      // Cardinal Compass  ======================================================================
      // 16-wind compass rose, calculated in the firmware
      std_msgs::msg::String compass_msg;
      compass_msg.data = p[10];

      headingGPS_pub_->publish(heading_msg);
      compassGPS_pub_->publish(compass_msg);

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "GPS_imu_node: Failed try in handle_gps(): %s", e.what());
    }
  }

  // --------------------------------------------------
  // IMU handler
  //   Format: IMU,ms,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz
  // --------------------------------------------------
  void handle_imu(const std::string & line)
  {
    // ACCEL_CONVERSION and GYRO_CONVERSION are done in the firmware (m/s^2 and rad/s)
    const double MAG_CONVERSION = 0.000001;  // it will be coming in micro Tesla, need Tesla
                                             // tested in arduino, need to convert here to preserve precision
    try {
      std::vector<std::string> p = split(line);
      if (p.size() != 15) {
        throw std::invalid_argument("expected 15 fields, got " + std::to_string(p.size()));
      }
      // p = _, ms, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz
      double ax = to_float(p[2]);
      double ay = to_float(p[3]);
      double az = to_float(p[4]);

      double gx = to_float(p[5]);  // should already be radians/second
      double gy = to_float(p[6]);
      double gz = to_float(p[7]);

      double mx = to_float(p[8]) * MAG_CONVERSION;
      double my = to_float(p[9]) * MAG_CONVERSION;
      double mz = to_float(p[10]) * MAG_CONVERSION;

      double qw = to_float(p[11]);
      double qx = to_float(p[12]);
      double qy = to_float(p[13]);
      double qz = to_float(p[14]);

      // IMU data ==============================================================================
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = this->now();
      imu_msg.header.frame_id = frame_id_;

      imu_msg.linear_acceleration.x = ax;
      imu_msg.linear_acceleration.y = ay;
      imu_msg.linear_acceleration.z = az;

      imu_msg.angular_velocity.x = gx;
      imu_msg.angular_velocity.y = gy;
      imu_msg.angular_velocity.z = gz;

      imu_msg.orientation.w = qw;
      imu_msg.orientation.x = qx;
      imu_msg.orientation.y = qy;
      imu_msg.orientation.z = qz;

      imu_msg.orientation_covariance = {
        1.0e-2, 0.0, 0.0,
        0.0, 1.0e-2, 0.0,
        0.0, 0.0, 1.0e-2};
      imu_msg.linear_acceleration_covariance = {
        4.5359e-4, 0.0, 0.0,
        0.0, 4.5359e-4, 0.0,
        0.0, 0.0, 1.81434e-3};
      imu_msg.angular_velocity_covariance = {
        9.98194e-6, 0.0, 0.0,
        0.0, 6.59392e-6, 0.0,
        0.0, 0.0, 4.71762e-6};

      imu_pub_->publish(imu_msg);

      // Magnetometer data ==========================================================================
      if (publish_mag_) {
        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = imu_msg.header.stamp;  // let's use same timestamp here
        mag_msg.header.frame_id = frame_id_;  // same frame ids for both mag and imu

        mag_msg.magnetic_field.x = mx;
        mag_msg.magnetic_field.y = my;
        mag_msg.magnetic_field.z = mz;
        mag_msg.magnetic_field_covariance = {
          1.0e-2, 0.0, 0.0,
          0.0, 1.0e-2, 0.0,
          0.0, 0.0, 1.0e-2};

        mag_pub_->publish(mag_msg);
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "imu_node: Failed try in handle_imu(): %s", e.what());
    }
  }

  // parameters
  std::string port_;
  int baud_;
  std::string frame_id_;
  bool simulated_data_;
  bool publish_mag_;

  // serial
  int serial_fd_;
  std::string serial_buffer_;

  // thread
  std::thread thread_;
  std::atomic<bool> running_{true};

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr NavSatFix_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr headingGPS_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compassGPS_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUGPSNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
