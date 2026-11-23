
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <future>
#include <string>
#include <vector>

namespace autonav_vision
{

class PanoramaCapture : public rclcpp::Node
{
public:
    PanoramaCapture();

private:
    // Sweep states
    enum class State
    {
        IDLE,
        MOVING,
        WAITING,
        CAPTURING
    };

    // Result returned by the image download worker
    struct CaptureResult
    {
        std::vector<unsigned char> bytes;
        std::string error;
    };

    // Metadata for each successfully captured frame
    struct FrameMetadata
    {
        std::string frame;
        double servo_angle;
        std::string image_path;
    };

    // ROS 2 callbacks
    void trigger_callback(
        const std_msgs::msg::String::SharedPtr msg);

    void update();

    // Panorama state machine
    void start_sweep(const std::string & folder_name);
    void move_to_next_angle();
    void start_capture();
    void finish_capture();
    void complete_sweep();
    void reset_sweep();

    // Image capture
    static CaptureResult fetch_snapshot(
        const std::string & url,
        long timeout_ms);

    static std::size_t write_bytes(
        char * data,
        std::size_t size,
        std::size_t nmemb,
        void * user_data);

    // File management
    std::filesystem::path create_sweep_folder(
        const std::string & folder_name);

    void save_metadata_csv();

    // ROS 2 interfaces
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        motor_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        trigger_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    bool test_mode_;
    std::string cam_url_;
    std::filesystem::path save_dir_;

    double start_angle_;
    double end_angle_;
    int num_angles_;

    double move_time_;
    double settle_time_;
    double capture_timeout_;

    // Sweep data
    State state_ = State::IDLE;

    std::vector<double> sweep_angles_;
    std::size_t sweep_index_ = 0;

    std::filesystem::path current_sweep_dir_;
    std::vector<FrameMetadata> frame_metadata_;

    // Motor settling deadline
    std::chrono::steady_clock::time_point ready_time_;

    // Asynchronous HTTP capture
    std::future<CaptureResult> capture_future_;
};

} 
