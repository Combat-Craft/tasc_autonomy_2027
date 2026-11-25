/*
  aruco_node.cpp - Aruco "Read and Store" / "Retrieve" node
  ROS 2 Jazzy, C++17, OpenCV 4.6+ (4.7+ API also supported)
 
 
  Purpose:
  
    Implements the two Aruco commands for the TASC/CIRC 2027 job-site board
    (5 markers in a single row, read left to right):
      - Read and Store: detect the 5 markers, display their IDs, remember them.
      - Retrieve: display the IDs from the most recent Read and Store.
    Both commands end in the same publish_stored() function, so "display after
    saving" and "display on command" behave identically.
    
    
    This implementation uses a single node rather than splitting into multiple. Reasons:
     - Shared state. The stored IDs and image are ordinary member variables,
       so "store" and "retrieve" see the same data with no extra topics or
       services just to pass state between nodes.
     - One display path. A single publish function serves both commands.
     - Low load. Detection on one frame is cheap, so there is no performance
       reason to split the work across processes.
   Trade-off: a crash in detection would also take down retrieval. If heavy
   work (like the network camera streaming) is added later, that part can be
   split into its own node without changing this node's topic interface.
    
 
  What it shows:
  
    - Log lines with the stored IDs, e.g. "Stored marker IDs (left to right):
      [7, 23, 42, 5, 88]", or a warning if nothing is saved / not 5 markers.
    - An OpenCV window with the saved frame, each marker outlined and labelled.
    - The same data on ROS topics (IDs as integers, and the image).
 
  Topics:
    Subscribes:
      /aruco/capture_cmd   std_msgs/String     trigger: Read and Store
      /aruco/retrieve_cmd  std_msgs/String     trigger: Retrieve
      <image_topic>        sensor_msgs/Image   camera frames
    Publishes:
      /aruco/ids           std_msgs/Int32MultiArray   stored IDs; empty = none saved
      /aruco/image         sensor_msgs/Image          stored frame, markers outlined
    (String triggers based off last yr's conventions from panorama_capture.py)
 
  Parameters:
    image_topic       default /camera/image_raw   camera topic to listen to
    test_image        default "" (unused)         path to a still image used as a fake frame
    dictionary        default DICT_4X4_100        marker family dictionary (maybe changing this competition? will ask)
    expected_markers  default 5                   markers required to save
    show_window       default true                false for headless runs
 

 
  Running/Testing:
    ros2 run aruco_task aruco_node --ros-args -p image_topic:=/image_raw
    ros2 topic pub --once /aruco/capture_cmd  std_msgs/msg/String "{data: go}"
    ros2 topic pub --once /aruco/retrieve_cmd std_msgs/msg/String "{data: go}"
    ros2 topic echo /aruco/ids
    No camera: add  -p test_image:=$HOME/aruco_test.png
 
  Common error messages:
    No frame yet or not exactly 5 markers -> warning, previously saved set kept.
    Retrieve with nothing saved -> warning, empty IDs array published.
 
  Current known limitations:
    - Uses one frame per command (a blurred frame can miss a marker).
    - No check that the camera is still publishing (stale frame is reused).
    - Left-to-right order assumes the camera is roughly level with the board.
    - Team IP camera serves HTTP snapshots, so a bridge to sensor_msgs/Image
      is still needed for the real rover.
    - Dictionary and marker size to be confirmed; "Press Marked Button"
      command not implemented yet.
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>   // use <cv_bridge/cv_bridge.h> if .hpp is not found
#include <opencv2/opencv.hpp>

// OpenCV 4.7 changed the aruco API, compile whichever matches the installed version
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  #define ARUCO_NEW_API 1
  #include <opencv2/objdetect/aruco_detector.hpp>
#else
  #define ARUCO_NEW_API 0
  #include <opencv2/aruco.hpp>
#endif

using namespace std::chrono_literals;

// Map the "dictionary" parameter text to OpenCV's constant.
static int dict_from_name(const std::string & n)
{
  if (n == "DICT_4X4_50")  return cv::aruco::DICT_4X4_50;
  if (n == "DICT_4X4_100") return cv::aruco::DICT_4X4_100;
  if (n == "DICT_5X5_100") return cv::aruco::DICT_5X5_100;
  if (n == "DICT_6X6_250") return cv::aruco::DICT_6X6_250;
  return cv::aruco::DICT_4X4_100;   // unknown name: fall back to default
}

class ArucoNode : public rclcpp::Node
{
public:
  ArucoNode() : Node("aruco_node")
  {
    // Parameters (declare, then read)
    declare_parameter<std::string>("image_topic", "/camera/image_raw");
    declare_parameter<std::string>("test_image", "");
    declare_parameter<std::string>("dictionary", "DICT_4X4_100");
    declare_parameter<int>("expected_markers", 5);
    declare_parameter<bool>("show_window", true);

    expected_    = get_parameter("expected_markers").as_int();
    show_window_ = get_parameter("show_window").as_bool();
    const std::string dict_name = get_parameter("dictionary").as_string();
    const int dict_id = dict_from_name(dict_name);

    // Detector (API depends on OpenCV version)
#if ARUCO_NEW_API
    detector_ = std::make_unique<cv::aruco::ArucoDetector>(
      cv::aruco::getPredefinedDictionary(dict_id), cv::aruco::DetectorParameters());
#else
    dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
    params_ = cv::aruco::DetectorParameters::create();
#endif

    // Optional fake camera: preload a still image as the "latest frame"
    const std::string test_image = get_parameter("test_image").as_string();
    if (!test_image.empty()) {
      latest_frame_ = cv::imread(test_image, cv::IMREAD_COLOR);
      if (latest_frame_.empty()) {
        RCLCPP_ERROR(get_logger(), "Could not load test_image: %s", test_image.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Using test image: %s", test_image.c_str());
      }
    }

    // Publishers
    ids_pub_   = create_publisher<std_msgs::msg::Int32MultiArray>("/aruco/ids", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/aruco/image", 10);

    // Subscribers. SensorDataQoS (best-effort) must match the camera driver,
    // otherwise no frames are received.
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("image_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });

    capture_sub_ = create_subscription<std_msgs::msg::String>(
      "/aruco/capture_cmd", 10,
      [this](std_msgs::msg::String::ConstSharedPtr) { read_and_store(); });

    retrieve_sub_ = create_subscription<std_msgs::msg::String>(
      "/aruco/retrieve_cmd", 10,
      [this](std_msgs::msg::String::ConstSharedPtr) { retrieve(); });

    // OpenCV windows only refresh while waitKey runs, so call it on a timer.
    if (show_window_) {
      gui_timer_ = create_wall_timer(30ms, []() { cv::waitKey(1); });
    }

    RCLCPP_INFO(get_logger(),
      "aruco_node ready (dictionary=%s, expecting %d markers, OpenCV %s)",
      dict_name.c_str(), expected_, CV_VERSION);
  }

private:
  // Keep only the newest frame; detection happens on command, not per frame.
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
    }
    // Default single-threaded executor: no mutex needed for latest_frame_.
  }

  // Command 1: Read and Store
  void read_and_store()
  {
    if (latest_frame_.empty()) {
      RCLCPP_WARN(get_logger(), "No camera frame yet, cannot read markers");
      return;
    }

    // Detect: corners holds 4 points per marker; ids is parallel to corners.
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
#if ARUCO_NEW_API
    detector_->detectMarkers(latest_frame_, corners, ids, rejected);
#else
    cv::aruco::detectMarkers(latest_frame_, dictionary_, corners, ids, params_, rejected);
#endif

    // Return before touching stored data, so a bad frame never wipes a good set.
    if (static_cast<int>(ids.size()) != expected_) {
      RCLCPP_WARN(get_logger(),
        "Saw %zu markers, need exactly %d. Keeping previously saved set.",
        ids.size(), expected_);
      return;
    }

    // Sort left to right by each marker's average corner x. We sort an index
    // list so ids and corners stay aligned.
    std::vector<size_t> order(ids.size());
    std::iota(order.begin(), order.end(), 0);

    auto center_x = [&](size_t i) {
      float s = 0.f;
      for (const auto & p : corners[i]) s += p.x;
      return s / 4.f;
    };
    std::sort(order.begin(), order.end(),
      [&](size_t a, size_t b) { return center_x(a) < center_x(b); });

    std::vector<int> sorted_ids;
    std::vector<std::vector<cv::Point2f>> sorted_corners;
    for (size_t i : order) {
      sorted_ids.push_back(ids[i]);
      sorted_corners.push_back(corners[i]);
    }

    // Save (overwrites the previous set). clone() because drawing modifies the
    // image and must not touch the live frame.
    stored_ids_ = sorted_ids;
    stored_image_ = latest_frame_.clone();
    cv::aruco::drawDetectedMarkers(stored_image_, sorted_corners, sorted_ids);

    RCLCPP_INFO(get_logger(), "Saved new marker set");
    publish_stored();
  }

  // Command 2: Retrieve
  void retrieve() { publish_stored(); }

  // Shared output path: publish IDs + image, log, and show the window.
  void publish_stored()
  {
    std_msgs::msg::Int32MultiArray ids_msg;

    // Nothing saved: publish an empty array so listeners can tell.
    if (stored_ids_.empty()) {
      RCLCPP_WARN(get_logger(), "No markers saved yet");
      ids_pub_->publish(ids_msg);
      return;
    }

    ids_msg.data = stored_ids_;
    ids_pub_->publish(ids_msg);

    // Header left empty (no timestamp); set one later if needed.
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", stored_image_).toImageMsg();
    image_pub_->publish(*img_msg);

    std::ostringstream oss;
    for (size_t i = 0; i < stored_ids_.size(); ++i) {
      oss << stored_ids_[i] << (i + 1 < stored_ids_.size() ? ", " : "");
    }
    RCLCPP_INFO(get_logger(), "Stored marker IDs (left to right): [%s]", oss.str().c_str());

    if (show_window_) cv::imshow("Stored ArUco markers", stored_image_);
  }

  // State
  int expected_{5};
  bool show_window_{true};
  cv::Mat latest_frame_;          // newest camera frame (or test image)
  cv::Mat stored_image_;          // saved frame with markers outlined
  std::vector<int> stored_ids_;   // saved IDs, left to right; empty = none saved

#if ARUCO_NEW_API
  std::unique_ptr<cv::aruco::ArucoDetector> detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
#endif

  // ROS handles must stay alive as members or the subscriptions stop.
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr capture_sub_, retrieve_sub_;
  rclcpp::TimerBase::SharedPtr gui_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoNode>());   // blocks until Ctrl+C
  rclcpp::shutdown();
  return 0;
}
