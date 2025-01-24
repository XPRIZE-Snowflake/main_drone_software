#include <chrono>
#include <cstdio>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Seek Thermal SDK (C includes)
extern "C" {
  #include <seekcamera/seekcamera.h>
  #include <seekcamera/seekcamera_manager.h>
  #include <seekcamera/seekcamera_error.h>
  #include <seekcamera/seekcamera_frame.h>
  #include <seekcamera/seekcamera_version.h>
  #include <seekframe/seekframe.h>
}

// Forward-declare callbacks
class SeekCameraNode;
static void cameraManagerEventCallback(
    seekcamera_t* camera,
    seekcamera_manager_event_t event,
    seekcamera_error_t status,
    void* user_data);

static void frameAvailableCallback(
    seekcamera_t* camera,
    seekcamera_frame_t* camera_frame,
    void* user_data);

// Minimal class to hold the node
class SeekCameraNode : public rclcpp::Node
{
public:
  SeekCameraNode()
  : Node("seekcamera_node")
  {
    // Create the ROS publisher
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/thermal_image", 10);

    // Create and init the Seek Camera Manager
    seekcamera_error_t res = seekcamera_manager_create(&manager_, SEEKCAMERA_IO_TYPE_USB);
    if (res != SEEKCAMERA_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "seekcamera_manager_create failed: %s",
        seekcamera_error_get_str(res));
      manager_ = nullptr;
      return;
    }

    // Register manager event callback
    res = seekcamera_manager_register_event_callback(
      manager_,
      cameraManagerEventCallback,
      /* user_data = */ this);
    if (res != SEEKCAMERA_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "seekcamera_manager_register_event_callback failed: %s",
        seekcamera_error_get_str(res));
      // Destroy manager if needed
      seekcamera_manager_destroy(&manager_);
      manager_ = nullptr;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Seek Camera Manager created and callback registered.");
    // The manager will now wait for cameras to connect
  }

  ~SeekCameraNode() override
  {
    // Cleanly destroy the manager on node destruction
    if (manager_) {
      seekcamera_manager_destroy(&manager_);
      manager_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Seek Camera Manager destroyed.");
    }
  }

  // This method is called by the static frame callback
  void publishCorrectedFrame(seekframe_t* frame)
  {
    if (!frame) {
      return;
    }

    size_t width  = seekframe_get_width(frame);
    size_t height = seekframe_get_height(frame);

    // Prepare a sensor_msgs::msg::Image
    sensor_msgs::msg::Image msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "seek_thermal_frame";

    msg.height = static_cast<uint32_t>(height);
    msg.width  = static_cast<uint32_t>(width);
    // 8-bit grayscale data
    msg.encoding = "mono8";
    msg.is_bigendian = false;
    // Each row has width
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width);

    // Resize data buffer for (height * width) bytes
    msg.data.resize(height * width);

    // Copy 8-bit data from the Seek frame
    void* frame_data = seekframe_get_data(frame);
    if (!frame_data) {
      return;
    }
    std::memcpy(msg.data.data(), frame_data, width * height);

    // Publish
    image_pub_->publish(msg);
  }

private:
  // Store the manager pointer so we can destroy it in the destructor
  seekcamera_manager_t* manager_{nullptr};

  // Store the publisher as a class member
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_{nullptr};
};

// -----------------------------
// Manager event callback (C-style)
// -----------------------------
static void cameraManagerEventCallback(
    seekcamera_t* camera,
    seekcamera_manager_event_t event,
    seekcamera_error_t status,
    void* user_data)
{
  auto node_ptr = static_cast<SeekCameraNode*>(user_data);
  if (!node_ptr) {
    return;
  }

  if (event == SEEKCAMERA_MANAGER_EVENT_CONNECT) {
    // Camera connected & paired
    if (status == SEEKCAMERA_SUCCESS) {
      // Start streaming CORRECTED frames (16-bit, minimal processing)
      seekcamera_error_t res = seekcamera_capture_session_start(
          camera,
          SEEKCAMERA_FRAME_FORMAT_PRE_AGC);
      if (res != SEEKCAMERA_SUCCESS) {
        fprintf(stderr,
                "[cameraManagerEventCallback] capture_session_start failed: %s\n",
                seekcamera_error_get_str(res));
      } else {
        // Register frame callback
        res = seekcamera_register_frame_available_callback(
            camera,
            frameAvailableCallback,
            /* user_data = */ node_ptr);
        if (res != SEEKCAMERA_SUCCESS) {
          fprintf(stderr,
                  "[cameraManagerEventCallback] register_frame_available_callback failed: %s\n",
                  seekcamera_error_get_str(res));
        } else {
          fprintf(stdout, "[cameraManagerEventCallback] CORRECTED (16-bit) streaming started.\n");
        }
      }
    }
  } else if (event == SEEKCAMERA_MANAGER_EVENT_DISCONNECT) {
    fprintf(stdout, "[cameraManagerEventCallback] Camera disconnected.\n");
    // You could call seekcamera_capture_session_stop(camera) here if desired
  } else if (event == SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR) {
    fprintf(stdout, "[cameraManagerEventCallback] Camera ready to pair (unpaired).\n");
  } else if (event == SEEKCAMERA_MANAGER_EVENT_ERROR) {
    fprintf(stderr, "[cameraManagerEventCallback] Camera error: %s\n",
            seekcamera_error_get_str(status));
  }
}

// -----------------------------
// Frame available callback (C-style)
// -----------------------------
static void frameAvailableCallback(
    seekcamera_t* camera,
    seekcamera_frame_t* camera_frame,
    void* user_data)
{
  (void)camera; // not used here

  auto node_ptr = static_cast<SeekCameraNode*>(user_data);
  if (!node_ptr || !camera_frame) {
    return;
  }

  // We want the CORRECTED frame (minimal processing)
  seekframe_t* frame_corrected = nullptr;
  seekcamera_error_t res = seekcamera_frame_get_frame_by_format(
      camera_frame,
      SEEKCAMERA_FRAME_FORMAT_CORRECTED,
      &frame_corrected
  );

  if (res != SEEKCAMERA_SUCCESS || !frame_corrected) {
    // Not found or error
    return;
  }

  // Let our node handle publishing
  node_ptr->publishCorrectedFrame(frame_corrected);
}

// -----------------------------
// Main
// -----------------------------
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SeekCameraNode>();

  // Spin in a single- or multi-threaded executor. This blocks until Ctrl-C or shutdown.
  rclcpp::spin(node);

  // After spin exits, shut down ROS (which will eventually destroy the node).
  rclcpp::shutdown();
  return 0;
}
