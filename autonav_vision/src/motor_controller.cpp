#include "autonav_vision/motor_controller.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace autonav_vision
{

// -------------------------------------------------
// motor_controller node
// -------------------------------------------------

MotorController::MotorController(): Node("motor_controller") {

    // Serial connection parameters
    // They can be overridden by using either the launch file (panorama.launch.py) or --ros-args
    // i.e. ros2 run autonav_vision motor_controller --ros-args -p serial_port:=/dev/ttyUSB0
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/nano");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);

    // Open and configure the serial connection to the Arduino.
    open_serial();

    // Subscribe to the /motor_cmd topic, which receives the target camera angles
    motor_subscription_ =
        this->create_subscription<std_msgs::msg::Float64>(
            "/motor_cmd",
            10,
            std::bind(&MotorController::motor_command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Motor controller ready");
}

// -------------------------------------------------
// Serial Connection Initialization
// -------------------------------------------------

// Open and configure the serial connection to the Arduino.
void MotorController::open_serial() {

    // Convert the numeric baud rate into the corresponding Linux termios baud rate constant.
    speed_t baud;

    switch (baud_rate_) {
        case 9600:
            baud = B9600;
            break;
        case 57600:
            baud = B57600;
            break;
        case 115200:
            baud = B115200;
            break;
        default:
            throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud_rate_));
    }

    // Open the serial device and obtain its file descriptor.
    //
    // O_RDWR    : Allow both reading and writing.
    // O_NOCTTY  : Prevent the device from becoming the controlling terminal.
    // O_CLOEXEC : Automatically close the descriptor if this process executes another program.
    //
    // If it returns a negative value, it means that the device could not be openned. (i.e. invalid path/permissions.
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);

    if (serial_fd_ < 0) {
        throw std::runtime_error("Failed to open serial port " + serial_port_ + ": " + std::strerror(errno));
    }

    try {
        // termios is used to store the configuration of the serial device
        // (baud rate, data bits, parity, stop bits, and input/output processing)
        struct termios tty {};

        // Read the current serial configuration before modifying it.
        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            throw std::runtime_error(
                "Failed to read serial settings: " +
                std::string(std::strerror(errno)));
        }

        // Configure raw serial communication.
        cfmakeraw(&tty);

        // Configure 8N1 serial communication:
        tty.c_cflag &= ~CSIZE;   // Clear existing data-bit size
        tty.c_cflag |= CS8;      // Set 8 data bits
        tty.c_cflag &= ~PARENB;  // Disable parity
        tty.c_cflag &= ~CSTOPB;  // Use 1 stop bit

        // Disable RTS/CTS hardware flow control. (Not needed by Arduino)
        tty.c_cflag &= ~CRTSCTS;

        // CLOCAL : Ignore modem control lines, such as carrier detect.
        // CREAD  : Enable the serial receiver.
        tty.c_cflag |= CLOCAL | CREAD;

        // Set the input and output baud rates to match the Arduino firmware's Serial.begin(115200) configuration.
        if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0)
        {
            throw std::runtime_error("Failed to set serial baud rate");
        }

        // Configure serial read behavior.
        tty.c_cc[VMIN] = 0; // A read does not require a minimum number of bytes before returning.
        tty.c_cc[VTIME] = 10; // Read timeout of 10 deciseconds (1 second)

        // Apply the updated configuration immediately.
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            throw std::runtime_error("Failed to configure serial port: " + std::string(std::strerror(errno)));
        }


        // Wait 2 seconds
        std::this_thread::sleep_for(
            std::chrono::seconds(2));

        // Discard any unread startup data from the Arduino.
        if (tcflush(serial_fd_, TCIFLUSH) != 0)
        {
            throw std::runtime_error("Failed to clear serial input: " + std::string(std::strerror(errno)));
        }
    }
    catch (...) {
        ::close(serial_fd_);
        serial_fd_ = -1;
        throw;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Connected to Arduino on %s at %d baud",
        serial_port_.c_str(),
        baud_rate_);
}

// -------------------------------------------------
// Motor Command Callback
// -------------------------------------------------

// Received message from /motor_cmd.
void MotorController::motor_command_callback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    // Reject NaN and infinity values for angle
    if (!std::isfinite(msg->data))
    {
        RCLCPP_WARN(this->get_logger(), "Received invalid motor angle");
        return;
    }

    // Round Float64 value to an integer value.
    double cmd = std::nearbyint(msg->data);

    // Clamp the camera angle to -120 to +120 degree range.
    if (cmd > 120.0) {
        RCLCPP_WARN( this->get_logger(), "Command %.0f exceeds max limit of 120, clamping", cmd);
        cmd = 120.0;
    }
    else if (cmd < -120.0) {
        RCLCPP_WARN(this->get_logger(), "Command %.0f exceeds min limit of -120, clamping", cmd);
        cmd = -120.0;
    }

    // Convert the validated angle from double to integer.
    const int camera_angle = static_cast<int>(cmd);

    // Map the camera angle range [-120, +120] to the Arduino Servo library's command range [0, 180].
    const int arduino_angle =
        (camera_angle + 120) * 180 / 240;

    // Send the mapped angle to the Arduino.
    if (send_serial_command(arduino_angle)) {
        RCLCPP_INFO(this->get_logger(), "Received: %d -> Sent to Arduino (mapped): %d", camera_angle, arduino_angle);
    }
}

// -------------------------------------------------
// Serial Command Transmission
// -------------------------------------------------

// Sends an integer servo command to the Arduino as a newline-terminated string.
bool MotorController::send_serial_command(int angle)
{
    // Convert the integer angle into an ASCII string and append a newline to indicate the end of the command.
    const std::string command = std::to_string(angle) + "\n";

    // Track how many bytes have been transmitted.
    std::size_t bytes_sent = 0;

    while (bytes_sent < command.size())
    {
        // Write the remaining command bytes to the serial device.
        const ssize_t result = ::write(
            serial_fd_,
            command.data() + bytes_sent,
            command.size() - bytes_sent);

        if (result < 0 && errno == EINTR) {
            continue;
        }

        // A negative or zero result mean an error or no bytes were written, respectively.
        if (result <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", result < 0 ? std::strerror(errno) : "No bytes written");
            return false;
        }

        bytes_sent += static_cast<std::size_t>(result);
    }

    return true;
}

// -------------------------------------------------
// Destroy node
// -------------------------------------------------

MotorController::~MotorController()
{
    // Only close the device if its file descriptor is valid.
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

}  

// entry point
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        // Create the MotorController node
        auto node = std::make_shared<autonav_vision::MotorController>();
        rclcpp::spin(node);
    }
    catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("motor_controller"), "Motor controller failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
