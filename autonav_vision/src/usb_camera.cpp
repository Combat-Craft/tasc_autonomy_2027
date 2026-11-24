#include <gst/app/gstappsink.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

using Image = sensor_msgs::msg::CompressedImage;

// deleting the gstreamer pipeline
struct DeletePipeline {
  void operator()(GstElement *pipeline) const {
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
  }
};

/*
 * @param:          rclcpp::Node::SharedPtr &node
 *                  the node for which we would like to take the video input in
 * and then encode the data values
 *
 * Description:     we are reading raw camera v412src at 640x480 at 30 fps
 *                  from /dev/video0 in VUY2 format and then converting it into
 *                  raw bgr frames of quality 80 and then it hands it off to the
 *                  appsink
 *                  the node name itself is called usb_camera
 *
 * Publisher:       we are publishing images on the topic name
 *                  image_raw/compressed this is happening every single 30
 *                  frames per seocnd the appsink will wait for basically 33.3
 *                  ms before sending the next package over
 *
 */
int run(const rclcpp::Node::SharedPtr &node) {
  const auto device =
      node->declare_parameter<std::string>("device", "/dev/video0");
  const int width = node->declare_parameter<int>("width", 640);
  const int height = node->declare_parameter<int>("height", 480);
  const int fps = node->declare_parameter<int>("fps", 30);
  const int quality = node->declare_parameter<int>("quality", 80);
  const int bitrate = node->declare_parameter<int>("bitrate_bps", 2000000);
  const auto encoder =
      node->declare_parameter<std::string>("encoder", "jpegenc");

  if (width <= 0 || height <= 0 || fps <= 0 || quality < 0 || quality > 100 ||
      bitrate <= 0) {
    throw std::runtime_error("Check filters.");
  }

  // encoding format which we are using
  std::string encoding;
  try {
    encoding = "videoconvert ! video/x-raw,format=BGR ! jpegenc quality=" +
               std::to_string(quality);
  } catch (const std::exception &e) {
    throw std::runtime_error("Invalid encoder.");
  }

  // the incomming without formatted video
  const std::string src = "v4l2src name=camera";
  const auto launch =
      src + " ! video/x-raw,format=YUY2,width=" + std::to_string(width) +
      ",height=" + std::to_string(height) +
      ",framerate=" + std::to_string(fps) + "/1 ! " + encoding +
      " ! image/jpeg ! appsink name=frames max-buffers=1 drop=true sync=false";

  GError *error = nullptr;
  std::unique_ptr<GstElement, DeletePipeline> pipeline(
      gst_parse_launch(launch.c_str(), &error));

  if (error) {
    const std::string why = error->message;
    g_error_free(error);
    throw std::runtime_error(why);
  }

  if (!pipeline)
    throw std::runtime_error("Could not create pipeline");

  if (!pipeline)
    throw std::runtime_error("Could not create pipeline");

  // getting the camera and then encoding the camera strem into bgr
  auto *camera = gst_bin_get_by_name(GST_BIN(pipeline.get()), "camera");
  if (!camera) {
    throw std::runtime_error("Could not find camera element");
  }
  g_object_set(camera, "device", device.c_str(), nullptr);
  gst_object_unref(camera);

  std::unique_ptr<GstElement, decltype(&gst_object_unref)> sink(
      gst_bin_get_by_name(GST_BIN(pipeline.get()), "frames"), gst_object_unref);
  std::unique_ptr<GstBus, decltype(&gst_object_unref)> bus(
      gst_element_get_bus(pipeline.get()), gst_object_unref);
  auto publisher = node->create_publisher<Image>(
      "image_raw/compressed", rclcpp::SensorDataQoS().keep_last(1));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto startup = gst_element_set_state(pipeline.get(), GST_STATE_PLAYING);

  while (rclcpp::ok()) {
    // error handelling
    auto *event = gst_bus_pop_filtered(
        bus.get(),
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (event) {
      if (GST_MESSAGE_TYPE(event) == GST_MESSAGE_ERROR) {
        gchar *debug = nullptr;
        gst_message_parse_error(event, &error, &debug);
        const std::string why = error->message;
        g_clear_error(&error);
        g_free(debug);
        gst_message_unref(event);
        throw std::runtime_error(why);
      }
      gst_message_unref(event);
      break;
    }

    if (startup == GST_STATE_CHANGE_FAILURE)
      throw std::runtime_error("Camera failed to start");
    executor.spin_some();
    std::unique_ptr<GstSample, decltype(&gst_sample_unref)> sample(
        gst_app_sink_try_pull_sample(GST_APP_SINK(sink.get()),
                                     100 * GST_MSECOND),
        gst_sample_unref);
    if (!sample)
      continue;

    auto *buffer = gst_sample_get_buffer(sample.get());
    if (!buffer)
      continue;
    Image msg;
    // Estimate capture time from the age of this sample in the live pipeline.
    // stamp -> time rn
    // segment the overall encoded data
    // pts -> for presentation timestamp when the media buffer should be
    // presented before playback
    auto stamp = node->now();
    auto *segment = gst_sample_get_segment(sample.get());
    auto pts = GST_BUFFER_PTS(buffer);
    if (segment && segment->format == GST_FORMAT_TIME &&
        GST_CLOCK_TIME_IS_VALID(pts)) {
      auto captured =
          gst_segment_to_running_time(segment, GST_FORMAT_TIME, pts);
      auto current = gst_element_get_current_running_time(pipeline.get());
      if (GST_CLOCK_TIME_IS_VALID(captured) &&
          GST_CLOCK_TIME_IS_VALID(current) && current >= captured) {
        auto age = current - captured;
        if (stamp.nanoseconds() > 0 &&
            age <= static_cast<GstClockTime>(stamp.nanoseconds()))
          stamp = stamp -
                  rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(age));
      }
    }
    msg.header.stamp = stamp;
    msg.format = "bgr8; jpeg compressed bgr8";
    msg.data.resize(gst_buffer_get_size(buffer));
    if (msg.data.empty() ||
        gst_buffer_extract(buffer, 0, msg.data.data(), msg.data.size()) !=
            msg.data.size())
      throw std::runtime_error("Failed to copy the JPEG frame");
    // publishing the message
    publisher->publish(std::move(msg));
  }
  return 0;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  gst_init(nullptr, nullptr);
  int status = 0;
  try {
    status = run(std::make_shared<rclcpp::Node>("usb_camera"));
  } catch (const std::exception &e) {
    std::cerr << "usb_camera: " << e.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
