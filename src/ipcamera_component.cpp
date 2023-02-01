// Copyright (c) 2019 Tasuku Miura
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #define RTV_BENCHMARK
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include "ros2_ipcamera/ipcamera_component.hpp"


namespace ros2_ipcamera
{
  IpCamera::IpCamera(const std::string & node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options),
    qos_(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()),
    transport_delay_(rclcpp::Duration::from_seconds(0))
  {
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "middleware: %s", rmw_get_implementation_identifier());

    // Declare parameters.
    this->initialize_parameters();

    this->configure();

    //TODO(Tasuku): add call back to handle parameter events.
    // Set up publishers.
    if (fast_mjpg_republishing_) {
      img_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("~/image_raw/compressed", qos_);
      info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos_);
    }
    else
      this->pub_ = image_transport::create_camera_publisher(
        this, "~/image_raw", qos_.get_rmw_qos_profile());

    this->captured_image_ = std::make_shared<cv::Mat>();
    this->capture_thread_ = std::thread(&IpCamera::capture, this);
    // execute must be performed in another thread to prevent blocking component container spinning
    // Note: a timer could be more elegant for this
    this->publish_thread_ = std::thread(&IpCamera::execute, this);
  }

  IpCamera::IpCamera(const rclcpp::NodeOptions & options)
  : IpCamera::IpCamera("ipcamera", options)
  {}

  IpCamera::~IpCamera() {
    this->capture_thread_.join();
    this->publish_thread_.join();
  }
  void
  IpCamera::configure()
  {
    rclcpp::Logger node_logger = this->get_logger();

    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    this->get_parameter<std::string>("rtsp_uri", source_);
    RCLCPP_INFO(node_logger, "rtsp_uri: %s", source_.c_str());

    this->get_parameter<std::string>("camera_calibration_file", camera_calibration_file_param_);
    RCLCPP_INFO(node_logger, "camera_calibration_file: %s",
                camera_calibration_file_param_.c_str());

    this->get_parameter<int>("image_width", width_);
    RCLCPP_INFO(node_logger, "image_width: %d", width_);

    this->get_parameter<int>("image_height", height_);
    RCLCPP_INFO(node_logger, "image_height: %d", height_);

    this->get_parameter<int>("image_height", height_);
    RCLCPP_INFO(node_logger, "image_height: %d", height_);

    this->get_parameter<std::string>("frame_id", frame_id_);
    RCLCPP_INFO(node_logger, "frame_id: %s", frame_id_.c_str());

    double transport_delay;
    this->get_parameter<double>("transport_delay", transport_delay);
    RCLCPP_INFO(node_logger, "transport_delay: %lf", transport_delay);
    transport_delay_ = rclcpp::Duration::from_seconds(transport_delay);

    this->get_parameter("rate", rate_);
    this->get_parameter("fast_mjpg_republishing", fast_mjpg_republishing_);

    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    if (fast_mjpg_republishing_)
      this->cap_.open(source_, cv::CAP_ANY, std::vector<int>({cv::CAP_PROP_FORMAT, -1}));
    else
      this->cap_.open(source_);

    // Set the width and height based on command line arguments.
    // The width, height has to match the available resolutions of the IP camera.
    this->cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    this->cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    if (!this->cap_.isOpened()) {
      RCLCPP_ERROR(node_logger, "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    // https://docs.ros.org/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html#_details
    // Make sure that cname is equal to camera_name in camera_info.yaml file. Default cname is set to "camera".
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    if (cinfo_manager_->validateURL(camera_calibration_file_param_)) {
      cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);
    } else {
      RCLCPP_WARN(node_logger, "CameraInfo URL not valid.");
      RCLCPP_WARN(node_logger, "URL IS %s", camera_calibration_file_param_.c_str());
    }
  }

  void
  IpCamera::initialize_parameters()
  {
    rcl_interfaces::msg::ParameterDescriptor rtsp_uri_descriptor;
    rtsp_uri_descriptor.name = "rtsp_uri";
    rtsp_uri_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    rtsp_uri_descriptor.description = "RTSP URI of the IP camera.";
    rtsp_uri_descriptor.additional_constraints = "Should be of the form 'rtsp://";
    this->declare_parameter("rtsp_uri", "", rtsp_uri_descriptor);

    rcl_interfaces::msg::ParameterDescriptor camera_calibration_file_descriptor;
    camera_calibration_file_descriptor.name = "camera_calibration_file";
    camera_calibration_file_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter(
      "camera_calibration_file", "", camera_calibration_file_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_width_descriptor;
    image_width_descriptor.name = "image_width";
    image_width_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_width", 640, image_width_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_height_descriptor;
    image_height_descriptor.name = "image_height";
    image_height_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_height", 480, image_height_descriptor);

    this->declare_parameter("frame_id", rclcpp::ParameterValue(std::string("camera_link")));
    this->declare_parameter("transport_delay", rclcpp::ParameterValue(0.0));

    this->declare_parameter("rate", rclcpp::ParameterValue(30.0));

    this->declare_parameter("fast_mjpg_republishing", rclcpp::ParameterValue(false));
  }

#ifdef RTV_BENCHMARK
  double cap_dur = 0;
  double retr_dur = 0;
  double conv_dur = 0;
  double pub_dur = 0;
  int cap_cnt = 0;
  int retr_cnt = 0;
  int conv_cnt = 0;
  int pub_cnt = 0;
#endif

  void IpCamera::capture()
  {
//    auto clock_type = get_clock()->get_clock_type();
    while (rclcpp::ok()) {
//       RCLCPP_INFO(get_logger(), "Grabbing: %lf", now().seconds());
      capture_mutex_.lock();
#ifdef RTV_BENCHMARK
      auto start = std::chrono::steady_clock::now();
#endif
      bool success = cap_.grab();
      capture_stamp_ = now() - transport_delay_;
#ifdef RTV_BENCHMARK
      auto end = std::chrono::steady_clock::now();
      double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
      auto &total_dur = cap_dur;
      auto &total_cnt = cap_cnt;
      if (floor(total_dur) != floor(total_dur + dur))
        RCLCPP_INFO(get_logger(), "cap: %lf %d", total_dur + dur, total_cnt + 1);
      total_dur += dur;
      total_cnt++;
#endif

//      double stamp = cap_.get(cv::CAP_PROP_POS_MSEC);
//      capture_stamp_ = rclcpp::Time((int64_t)(stamp*1000000), clock_type);
      capture_mutex_.unlock();
//      RCLCPP_INFO(get_logger(), "Grabbed: %lf", capture_stamp_.seconds());
      if (!success) {
        RCLCPP_INFO(get_logger(), "Failed to capture a frame");
        rclcpp::Rate(rate_).sleep();
      }
      else
        rclcpp::sleep_for(std::chrono::milliseconds(1));  // give retrieve thread a chance to enter the mutex... (TODO: more elegant solution?)
    }
  }

  void
  IpCamera::execute()
  {
    rclcpp::Rate loop_rate(rate_);

    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());

    // Initialize OpenCV image matrices.
    
    // Our main event loop will spin until the user presses CTRL-C to exit.
    cv::Mat frame;
    rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    while (rclcpp::ok()) {
      // Initialize a shared pointer to an Image message.
      msg->is_bigendian = false;

      // Get the frame from the video capture.
      bool success = false;
      // RCLCPP_INFO(get_logger(), "Retrieving: %lf/%lf", capture_stamp_.seconds(), stamp.seconds());
      capture_mutex_.lock();
      // RCLCPP_INFO(get_logger(), "In lock");
      if (capture_stamp_.nanoseconds() != stamp.nanoseconds()) {
#ifdef RTV_BENCHMARK
        auto start = std::chrono::steady_clock::now();
#endif        
        success = cap_.retrieve(frame);
        stamp = capture_stamp_;
#ifdef RTV_BENCHMARK
        auto end = std::chrono::steady_clock::now();
        double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
        auto &total_dur = retr_dur;
        auto &total_cnt = retr_cnt;
        if (floor(total_dur) != floor(total_dur + dur))
          RCLCPP_INFO(get_logger(), "retr: %lf %d", total_dur + dur, total_cnt + 1);
        total_dur += dur;
        total_cnt++;
#endif
      }
      capture_mutex_.unlock();
//      RCLCPP_INFO(get_logger(), "Retrieved %lf", stamp.seconds());

      // Check if the frame was grabbed correctly
      if (success) {
        if (fast_mjpg_republishing_)
          publish_fast(frame, frame_id_, stamp, camera_info_msg);
        else {
          // Convert to a ROS image
          convert_frame_to_message(frame, frame_id_, stamp, *msg, *camera_info_msg);
          // Publish the image message and increment the frame_id.
#ifdef RTV_BENCHMARK
          auto start = std::chrono::steady_clock::now();
#endif
          this->pub_.publish(std::move(msg), camera_info_msg);
          msg = std::make_unique<sensor_msgs::msg::Image>();
#ifdef RTV_BENCHMARK
          auto end = std::chrono::steady_clock::now();
          double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
          auto &total_dur = pub_dur;
          auto &total_cnt = pub_cnt;
          if (floor(total_dur) != floor(total_dur + dur))
            RCLCPP_INFO(get_logger(), "pub: %lf %d", total_dur + dur, total_cnt + 1);
          total_dur += dur;
          total_cnt++;
#endif
        }
      }
      loop_rate.sleep();
    }
  }

  std::string
  IpCamera::mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("Unsupported encoding type");
    }
  }

  void
  IpCamera::convert_frame_to_message(
    const cv::Mat & frame,
    std::string frame_id,
    rclcpp::Time stamp,
    sensor_msgs::msg::Image & msg,
    sensor_msgs::msg::CameraInfo & camera_info_msg)
  {
#ifdef RTV_BENCHMARK
    auto start = std::chrono::steady_clock::now();
#endif
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);

    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    camera_info_msg.header.frame_id = frame_id;
    camera_info_msg.header.stamp = stamp;

#ifdef RTV_BENCHMARK
    auto end = std::chrono::steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    auto &total_dur = conv_dur;
    auto &total_cnt = conv_cnt;
    if (floor(total_dur) != floor(total_dur + dur))
      RCLCPP_INFO(get_logger(), "conv: %lf %d", total_dur + dur, total_cnt + 1);
    total_dur += dur;
    total_cnt++;
#endif
  }

  void IpCamera::publish_fast(
      const cv::Mat & frame,
      std::string frame_id,
      rclcpp::Time stamp,
      const sensor_msgs::msg::CameraInfo::SharedPtr &cam_info
    )
  {
#ifdef RTV_BENCHMARK
    auto start = std::chrono::steady_clock::now();
#endif
    cam_info->header.frame_id = frame_id;
    cam_info->header.stamp = stamp;

    auto img_msg_loan = img_pub_->borrow_loaned_message();
    auto &msg = img_msg_loan.get();
    msg.format = "bgr8; jpeg compressed bgr8";
    size_t size = frame.cols*frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
#ifdef RTV_BENCHMARK
    {
      auto end = std::chrono::steady_clock::now();
      double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
      auto &total_dur = conv_dur;
      auto &total_cnt = conv_cnt;
      if (floor(total_dur) != floor(total_dur + dur))
        RCLCPP_INFO(get_logger(), "conv: %lf %d", total_dur + dur, total_cnt + 1);
      total_dur += dur;
      total_cnt++;
    }
    start = std::chrono::steady_clock::now();
#endif
    img_pub_->publish(std::move(img_msg_loan));
    info_pub_->publish(*cam_info);
#ifdef RTV_BENCHMARK
    auto end = std::chrono::steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    auto &total_dur = pub_dur;
    auto &total_cnt = pub_cnt;
    if (floor(total_dur) != floor(total_dur + dur))
      RCLCPP_INFO(get_logger(), "pub: %lf %d", total_dur + dur, total_cnt + 1);
    total_dur += dur;
    total_cnt++;
#endif
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_ipcamera::IpCamera)
