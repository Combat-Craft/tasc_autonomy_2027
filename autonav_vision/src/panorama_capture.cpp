/*
 * ROS 2 Panorama Capture Node
 *
 * Workflow:
 *   1. Wait for a String message on /panorama_trigger (folder name).
 *   2. Create a unique output folder under save_dir.
 *   3. Publish each requested position on /motor_cmd.
 *   4. Wait for the servo to move and settle.
 *   5. Capture image with IP cam, and then download and save a JPEG snapshot (skipped in test mode)
 *   6. Write metadata.csv, return the servo to center, and become idle.
 *
 * State transitions:
 *   IDLE -> MOVING -> WAITING -> CAPTURING -> MOVING -> ... -> IDLE
 *   Skip CAPTURING in test mode
 *
 
 */

#include "autonav_vision/panorama_capture.hpp"

#include <cstdlib>
#include <curl/curl.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace fs = std::filesystem;

namespace {

// Expand to home directory
std::string expand_home(const std::string & path) {
    if (path == "~" || path.rfind("~/", 0) == 0) {
        const char * home = std::getenv("HOME");

        if (!home) {
            throw std::runtime_error("HOME is not set");
        }

        return std::string(home) + path.substr(1);
    }

    return path;
}

// Remove whitespace from the beginning and end
std::string trim(const std::string & input)
{
    const auto start = input.find_first_not_of(" \t\n\r");

    if (start == std::string::npos) {
        return "";
    }

    const auto end = input.find_last_not_of(" \t\n\r");
    return input.substr(start, end - start + 1);
}

// Replace unsafe folder-name characters
std::string sanitize_folder_name(const std::string & name)
{
    std::string result;

    // Restrict folder names to letters, digits, underscores and hyphens.
    for (unsigned char c : name)
    {
        if (std::isalnum(c) || c == '_' || c == '-') {
            result += static_cast<char>(c);
        }
        else {
            result += '_';
        }

    }

    return result;
}

// Escape special characters in CSV text
std::string csv_escape(const std::string & value) {
    if (value.find_first_of(",\"\n\r") == std::string::npos) {
        return value;
    }

    std::string escaped = "\"";

    for (char c : value) {
        if (c == '"') {
            escaped += "\"\"";
        }
        else {
            escaped += c;
        }
    }

    return escaped + "\"";
}

} 

namespace autonav_vision{

// panorama_capture node
PanoramaCapture::PanoramaCapture()
    : Node("panorama_capture")
{
    // -------------------------------------------------
    // Parameters
    // -------------------------------------------------

    test_mode_ = this->declare_parameter<bool>("test_mode", false);
    cam_url_ = this->declare_parameter<std::string>("cam_url", "http://192.168.1.117:6688/snapshot/PROFILE_000");

    // Parent directory for all triggered sweep folders.
    const std::string save_path = this->declare_parameter<std::string>("save_dir", "~/panorama_images");

    save_dir_ = fs::absolute(expand_home(save_path));

    // Sweep configuration [-120, +120].
    start_angle_ = this->declare_parameter<double>("start_angle", -120.0);
    end_angle_ = this->declare_parameter<double>("end_angle", 120.0);
    num_angles_ = this->declare_parameter<int>("num_angles", 12);

    // Timing configuration (seconds)
    // Pause after moving to each position
    // Total wait = move_time + settle_time
    move_time_ = this->declare_parameter<double>("move_time", 1.0); // 1 sec
    settle_time_ = this->declare_parameter<double>("settle_time", 1.0); // 1 sec

    // Maximum wall-clock time allowed for an HTTP snapshot request.
    capture_timeout_ = this->declare_parameter<double>("capture_timeout", 3.0);

    // Fail early for:
    // NaN/infinity
    // reversed/out-of-range angles
    // too few images
    // negative delays
    // unreasonable capture timeout
    if (!std::isfinite(start_angle_) ||
        !std::isfinite(end_angle_) ||
        start_angle_ >= end_angle_ ||
        start_angle_ < -120.0 ||
        end_angle_ > 120.0 ||
        num_angles_ < 2 ||
        !std::isfinite(move_time_) ||
        !std::isfinite(settle_time_) ||
        !std::isfinite(capture_timeout_) ||
        move_time_ < 0.0 ||
        settle_time_ < 0.0 ||
        capture_timeout_ <= 0.0 ||
        capture_timeout_ > 120.0)
    {
        throw std::runtime_error("Invalid panorama sweep configuration");
    }

    // A camera URL is necessary when real captures are enabled. (test mode is disabled)
    if (!test_mode_ && cam_url_.empty()) {
        throw std::runtime_error( "cam_url cannot be empty outside test mode");
    }

    // Create base output directory
    fs::create_directories(save_dir_);

    // Generate evenly spaced sweep angles
    sweep_angles_.reserve(static_cast<std::size_t>(num_angles_));

    for (int i = 0; i < num_angles_; ++i)
    {
        // Example: 12 positions divide the range into 11 equal intervals.
        const double fraction = static_cast<double>(i) / (num_angles_ - 1);

        // Linear interpolation between the first and last camera angles.
        const double angle = start_angle_ + (end_angle_ - start_angle_) * fraction;

        sweep_angles_.push_back(angle);
    }

    // -------------------------------------------------
    // ROS 2 publisher
    // -------------------------------------------------

    // Publish a camera angle.
    // motor_controller subscribes and converts it to the Arduino servo command.
    motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/motor_cmd", 10);

    // -------------------------------------------------
    // ROS 2 subscriber
    // -------------------------------------------------

    // A String on /panorama_trigger supplies the desired sweep folder name.
    trigger_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            "/panorama_trigger",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                trigger_callback(msg);
            });


    // -------------------------------------------------
    // Timer: 0.1-second update interval
    // -------------------------------------------------

    using namespace std::chrono_literals;

    // Execute the state-machine update every 100 ms without blocking other ROS 2 callbacks between positions.
    timer_ = this->create_wall_timer(
        100ms,
        [this]()
        {
            update();
        });

    RCLCPP_INFO(
        this->get_logger(),
        "TEST_MODE = %s",
        test_mode_ ? "true" : "false");

    RCLCPP_INFO(
        this->get_logger(),
        "IP Camera URL: %s",
        cam_url_.c_str());

    RCLCPP_INFO(
        this->get_logger(),
        "Base save directory: %s",
        save_dir_.string().c_str());

    RCLCPP_INFO(
        this->get_logger(),
        "Panorama ready: %d angles from %.1f to %.1f degrees",
        num_angles_,
        start_angle_,
        end_angle_);

    RCLCPP_INFO(
        this->get_logger(),
        "Send a trigger to /panorama_trigger");
}

// -------------------------------------------------
// Trigger callback
// -------------------------------------------------

// Only one sweep is active at a time. Incoming triggers during an ongoing sweep are ignored rather than queued.
void PanoramaCapture::trigger_callback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (state_ != State::IDLE) {
        RCLCPP_WARN(
            this->get_logger(),
            "Sweep already in progress, ignoring trigger");
        return;
    }

    // Remove whitespace;
    const std::string folder_name = trim(msg->data);

    if (folder_name.empty()) {
        RCLCPP_WARN( this->get_logger(), "Empty folder name received");
        return;
    }

    start_sweep(folder_name);
}

// -------------------------------------------------
// Create a unique sweep folder
// -------------------------------------------------
fs::path PanoramaCapture::create_sweep_folder(const std::string & folder_name) {
    // Normalize the requested name before using it as a directory path.
    std::string safe_name =
        sanitize_folder_name(folder_name);

    if (safe_name.empty()) {
        safe_name = "sweep";
    }

    fs::path folder = save_dir_ / safe_name;

    int counter = 1;

    while (fs::exists(folder)) {
        // Produce a zero-padded three-digit suffix (i.e. 001).
        std::ostringstream suffix;

        suffix << safe_name << "_"
               << std::setw(3)
               << std::setfill('0')
               << counter++;

        folder = save_dir_ / suffix.str();
    }

    // Create only the chosen sweep directory
    if (!fs::create_directory(folder)) {
        throw std::runtime_error(
            "Could not create sweep folder");
    }

    return folder;
}

// -------------------------------------------------
// Start panorama sweep
// -------------------------------------------------

// A valid trigger initializes a new sweep, resets its recorded data, and schedules the first motor movement on the next timer callback.
void PanoramaCapture::start_sweep(const std::string & folder_name) {
    try {
        // Make an isolated directory for this particular sweep so old images and metadata cannot be overwritten.
        current_sweep_dir_ = create_sweep_folder(folder_name);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to create sweep folder: %s",
            e.what());
        return;
    }

    // Begin at the first sweep position.
    sweep_index_ = 0;

    // Metadata must not carry over from the previous panorama.
    frame_metadata_.clear();

    state_ = State::MOVING;

    RCLCPP_INFO(
        this->get_logger(),
        "Starting sweep: %s",
        current_sweep_dir_.string().c_str());

    if (motor_pub_->get_subscription_count() == 0)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "No /motor_cmd subscriber detected");
    }
}

// -------------------------------------------------
// Main timer-driven state machine
// -------------------------------------------------

void PanoramaCapture::update()
{
    switch (state_)
    {
        // Nothing to do until a panorama trigger arrives.
        case State::IDLE:
            return;

        // Send the next position command or finish if all are done.
        case State::MOVING:
            move_to_next_angle();
            break;

        // Wait until both the commanded motion and settling delay elapse.
        case State::WAITING:
        {
            if (std::chrono::steady_clock::now() < ready_time_) {
                break;
            }

            // Test mode: move motor but skip real images
            if (test_mode_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[TEST] Simulated capture at %.2f degrees (%zu/%zu)",
                    sweep_angles_[sweep_index_],
                    sweep_index_ + 1,
                    sweep_angles_.size());

                ++sweep_index_;
                state_ = State::MOVING;
            }
            else {
                start_capture();
            }

            break;
        }

        // A camera request is currently running on a worker thread.
        case State::CAPTURING: {
            using namespace std::chrono_literals;

            if (capture_future_.valid() && capture_future_.wait_for(0ms) == std::future_status::ready) {
                finish_capture();
            }

            break;
        }
    }
}

// -------------------------------------------------
// Command next servo angle
// -------------------------------------------------

// Publish a Float64 angle to /motor_cmd.
void PanoramaCapture::move_to_next_angle()
{
    // When all positions are processed, write metadata and re-center.
    if (sweep_index_ >= sweep_angles_.size()) {
        complete_sweep();
        return;
    }

    // Current requested camera angle
    const double angle = sweep_angles_[sweep_index_];

    std_msgs::msg::Float64 msg;
    msg.data = angle;

    // Send one command for this position
    motor_pub_->publish(msg);

    RCLCPP_INFO(
        this->get_logger(),
        "Moving to %.2f degrees (%zu/%zu)",
        angle,
        sweep_index_ + 1,
        sweep_angles_.size());

    // Wait for motor movement and settling
    const auto delay = std::chrono::duration<double>(
        move_time_ + settle_time_);

    ready_time_ =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(delay);

    // Next timer ticks monitor the deadline before capture begins.
    state_ = State::WAITING;
}

// -------------------------------------------------
// Start asynchronous IP camera capture
// -------------------------------------------------

// The timer keeps running while the worker waits for the HTTP response.
void PanoramaCapture::start_capture()
{
    const long timeout_ms =
        static_cast<long>(capture_timeout_ * 1000.0);

    try {
       
        capture_future_ = std::async(
            std::launch::async,
            // Capture URL and timeout by value, so the worker does not
            // depend on the callback's local variables remaining alive.
            [url = cam_url_, timeout_ms]()
            {
                return PanoramaCapture::fetch_snapshot(
                    url, timeout_ms);
            });

        state_ = State::CAPTURING;
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not start image capture: %s",
            e.what());

        ++sweep_index_;
        state_ = State::MOVING;
    }
}

// -------------------------------------------------
// libcurl download callback
// -------------------------------------------------

std::size_t PanoramaCapture::write_bytes(
    char * data,
    std::size_t size,
    std::size_t nmemb,
    void * user_data)
{
    auto * bytes = static_cast<std::vector<unsigned char> *>(user_data);

    const std::size_t count = size * nmemb;

    // Limit HTTP response size to 20 MB
    constexpr std::size_t max_size = 20 * 1024 * 1024;

    // Reject another chunk if appending it would exceed the limit.
    if (bytes->size() > max_size || count > max_size - bytes->size()) {
        return 0;
    }

    try {
        const auto * begin = reinterpret_cast<const unsigned char *>(data);

        // Store the received binary JPEG data
        bytes->insert(
            bytes->end(),
            begin,
            begin + count);
    }
    catch (...) {
        return 0;
    }

    return count;
}

// -------------------------------------------------
// Download image from IP camera
// -------------------------------------------------

PanoramaCapture::CaptureResult
PanoramaCapture::fetch_snapshot(
    const std::string & url,
    long timeout_ms)
{
    CaptureResult result;

    CURL * curl = curl_easy_init();

    if (!curl) {
        result.error = "Failed to initialize libcurl";
        return result;
    }

    // Select the camera snapshot endpoint.
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    curl_easy_setopt(
        curl, CURLOPT_WRITEFUNCTION,
        &PanoramaCapture::write_bytes);

    curl_easy_setopt(
        curl, CURLOPT_WRITEDATA, &result.bytes);

    curl_easy_setopt(
        curl, CURLOPT_TIMEOUT_MS, timeout_ms);

    curl_easy_setopt(
        curl, CURLOPT_CONNECTTIMEOUT_MS,
        std::min(timeout_ms, 2000L));

    curl_easy_setopt(
        curl, CURLOPT_NOSIGNAL, 1L);

    curl_easy_setopt(
        curl, CURLOPT_FAILONERROR, 1L);

    curl_easy_setopt(
        curl, CURLOPT_FOLLOWLOCATION, 1L);

    curl_easy_setopt(
        curl, CURLOPT_MAXREDIRS, 3L);

    curl_easy_setopt(
        curl, CURLOPT_PROTOCOLS_STR, "http,https");

    curl_easy_setopt(
        curl, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");

    const CURLcode status = curl_easy_perform(curl);

    if (status != CURLE_OK)
    {
        result.error = curl_easy_strerror(status);
    }
    else if (result.bytes.empty())
    {
        result.error = "Camera returned an empty response";
    }

    // Release the per-request network resources in all outcomes.
    curl_easy_cleanup(curl);

    return result;
}

// -------------------------------------------------
// Decode and save captured image
// -------------------------------------------------

void PanoramaCapture::finish_capture() {
    try{

        CaptureResult result = capture_future_.get();

        if (!result.error.empty()) {
            throw std::runtime_error(result.error);
        }

        // Decode compressed image bytes
        cv::Mat frame = cv::imdecode(result.bytes, cv::IMREAD_COLOR);

        if (frame.empty()) {
            throw std::runtime_error("Failed to decode camera image");
        }

        std::ostringstream filename;

        filename << "frame_"
                 << std::setw(3)
                 << std::setfill('0')
                 << sweep_index_
                 << ".jpg";

        // Save inside the unique sweep directory created at trigger time.
        const fs::path image_path = current_sweep_dir_ / filename.str();

        // Save JPEG image
        if (!cv::imwrite(image_path.string(), frame)) {
            throw std::runtime_error("Failed to save image");
        }

        // Record metadata
        frame_metadata_.push_back({
            filename.str(),
            sweep_angles_[sweep_index_],
            image_path.string()
        });

        RCLCPP_INFO(
            this->get_logger(),
            "Saved image at %.2f degrees (%zu/%zu): %s",
            sweep_angles_[sweep_index_],
            sweep_index_ + 1,
            sweep_angles_.size(),
            image_path.string().c_str());
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Capture failed at angle %.2f: %s",
            sweep_angles_[sweep_index_],
            e.what());
    }

    // Continue even if one frame failed
    ++sweep_index_;
    state_ = State::MOVING;
}

// -------------------------------------------------
// Save metadata.csv
// -------------------------------------------------

void PanoramaCapture::save_metadata_csv()
{
    const fs::path metadata_path = current_sweep_dir_ / "metadata.csv";

    // Opens in output mode, replacing any existing file of this name.
    std::ofstream file(metadata_path);

    if (!file) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not create metadata.csv");
        return;
    }

    file << "frame,servo_angle,gps_latitude,"
            "gps_longitude,rover_heading,"
            "camera_heading,camera_cardinal,"
            "image_path\n";

    for (const auto & item : frame_metadata_) {
        file << csv_escape(item.frame) << ","
             << std::fixed << std::setprecision(4)
             << item.servo_angle
             // Five blank CSV fields between servo_angle and image_path.
             << ",,,,,,"
             << csv_escape(item.image_path)
             << "\n";
    }

    file.close();

    RCLCPP_INFO(
        this->get_logger(),
        "Saved metadata: %s",
        metadata_path.string().c_str());
}

// -------------------------------------------------
// Finish sweep
// -------------------------------------------------

void PanoramaCapture::complete_sweep()
{
    RCLCPP_INFO(
        this->get_logger(),
        "Sweep complete: %zu/%zu images saved",
        frame_metadata_.size(),
        sweep_angles_.size());

    save_metadata_csv();

    // Return servo to center
    std_msgs::msg::Float64 center_msg;
    center_msg.data = 0.0;

    motor_pub_->publish(center_msg);

    RCLCPP_INFO(
        this->get_logger(),
        "Returning servo to 0 degrees");

    reset_sweep();
}

// -------------------------------------------------
// Reset sweep state
// -------------------------------------------------

void PanoramaCapture::reset_sweep()
{
    sweep_index_ = 0;
    frame_metadata_.clear();

    // Forget the active folder path
    current_sweep_dir_.clear();

    // Only in IDLE will the trigger callback accept a new sweep.
    state_ = State::IDLE;

    RCLCPP_INFO(
        this->get_logger(),
        "Panorama capture ready for next trigger");
}

} 

int main(int argc, char * argv[])
{
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        std::cerr << "Failed to initialize libcurl\n";
        return 1;
    }

    rclcpp::init(argc, argv);

    int exit_code = 0;

    try {
        auto node = std::make_shared<autonav_vision::PanoramaCapture>();

        rclcpp::spin(node);
    }
    catch (const std::exception & e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("panorama_capture"),
            "Panorama capture failed: %s",
            e.what());

        exit_code = 1;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    curl_global_cleanup();

    return exit_code;
}
