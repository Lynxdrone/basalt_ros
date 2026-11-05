// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef BASALT_ROS__IMAGE_SUBSCRIBER_HPP_
#define BASALT_ROS__IMAGE_SUBSCRIBER_HPP_

#include <basalt/optical_flow/optical_flow.h>
#include <tbb/concurrent_queue.h>

#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

namespace basalt_ros
{
class ImageSubscriber
{
  typedef sensor_msgs::msg::Image Image;
  typedef std::shared_ptr<const Image> ImageConstPtr;
  typedef tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>
    OpticalFlowInputQueue;
  typedef std::shared_ptr<OpticalFlowInputQueue> OpticalFlowInputQueuePtr;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // one topic per camera, in calibration order. Images are grouped into a
  // frame when their stamps are within sync_tolerance of the first camera's
  ImageSubscriber(
    rclcpp::Node * node, const std::vector<std::string> & topics,
    int64_t sync_tolerance_ns);

  ~ImageSubscriber() {}

  OpticalFlowInputQueue * getQueue() { return queue_; }
  void setQueue(OpticalFlowInputQueue * q) { queue_ = q; }

private:
  void imageCallback(size_t cam, const ImageConstPtr & img);
  void callback(const std::vector<ImageConstPtr> & msg);
  // ----- variables --
  rclcpp::Node * node_;
  std::vector<rclcpp::Subscription<Image>::SharedPtr> subs_;
  int64_t syncToleranceNs_{0};
  std::mutex mutex_;
  // incomplete frames by stamp of the first image received
  std::map<int64_t, std::vector<ImageConstPtr>> pending_;
  int64_t lastFrameNs_{-1};  // stamp of the last frame sent to the queue
  OpticalFlowInputQueue * queue_{nullptr};
  int64_t max_q_{0};
  uint64_t framesReceived_{0};
  rclcpp::Time lastTime_;
};
}  // namespace basalt_ros
#endif  //  BASALT_ROS__IMAGE_SUBSCRIBER_HPP_
