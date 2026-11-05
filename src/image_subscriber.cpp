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

#include "basalt_ros/image_subscriber.hpp"

#include <cstdlib>
#include <functional>
#include <stdexcept>

namespace basalt_ros
{
// frames kept waiting for their missing images
static constexpr size_t MAX_PENDING_FRAMES = 100;

static int64_t to_ns(const builtin_interfaces::msg::Time & t)
{
  return t.sec * 1000000000LL + t.nanosec;
}

ImageSubscriber::ImageSubscriber(
  rclcpp::Node * node, const std::vector<std::string> & topics,
  int64_t sync_tolerance_ns)
: node_(node), syncToleranceNs_(sync_tolerance_ns)
{
  for (size_t cam = 0; cam < topics.size(); cam++) {
    subs_.push_back(node_->create_subscription<Image>(
      topics[cam], 10, [this, cam](const ImageConstPtr msg) {
        imageCallback(cam, msg);
      }));
  }
  lastTime_ = node_->now();
}

void ImageSubscriber::imageCallback(size_t cam, const ImageConstPtr & img)
{
  std::vector<ImageConstPtr> frame;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t t = to_ns(img->header.stamp);
    if (t <= lastFrameNs_) {
      // basalt needs strictly increasing frame stamps (the ZED sim repeats some)
      RCLCPP_WARN_STREAM_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "dropping image " << t << " of cam " << cam
                          << ", not newer than last frame " << lastFrameNs_);
      return;
    }
    auto it = pending_.end();
    for (auto p = pending_.begin(); p != pending_.end(); ++p) {
      if (std::llabs(p->first - t) <= syncToleranceNs_ && !p->second[cam]) {
        it = p;
        break;
      }
    }
    if (it == pending_.end()) {
      it = pending_.emplace(t, std::vector<ImageConstPtr>(subs_.size())).first;
      if (it->second[cam]) {
        return;  // same stamp twice from this camera
      }
    }
    it->second[cam] = img;
    for (const auto & m : it->second) {
      if (!m) {
        if (pending_.size() > MAX_PENDING_FRAMES) {
          pending_.erase(pending_.begin());
        }
        return;
      }
    }
    frame = it->second;
    lastFrameNs_ = std::max(lastFrameNs_, to_ns(frame[0]->header.stamp));
    // older frames can no longer complete
    pending_.erase(pending_.begin(), std::next(it));
  }
  callback(frame);
}

void ImageSubscriber::callback(const std::vector<ImageConstPtr> & msg)
{
  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

  const size_t NUM_CAMS = msg.size();
  data->img_data.resize(NUM_CAMS);
  data->t_ns = to_ns(msg[0]->header.stamp);
  for (size_t i = 0; i < NUM_CAMS; i++) {
    const auto & img = *msg[i];
    data->img_data[i].exposure =
      0;  /// XXX  RS2_FRAME_METADATA_ACTUAL_EXPOSURE * 1e-6

    data->img_data[i].img.reset(
      new basalt::ManagedImage<uint16_t>(img.width, img.height));

    uint16_t * data_out = data->img_data[i].img->ptr;
    // TODO(Bernd): this 8-bit to 16bit conversion can probably be done
    // more efficiently with opencv
    for (size_t y = 0; y < img.height; y++) {
      const uint8_t * data_in = &img.data[y * img.step];
      for (size_t x = 0; x < img.width; x++) {
        data_out[y * img.width + x] = data_in[x] << 8;
      }
    }
  }

  framesReceived_++;
  if (framesReceived_ == 100) {
    const auto t = node_->now();
    const auto dt = t - lastTime_;
    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "received imag frame rate: " << framesReceived_ * 1e9 / dt.nanoseconds());
    framesReceived_ = 0;
    lastTime_ = t;
  }

  if (queue_) {
    if (!queue_->try_push(data)) {
      RCLCPP_WARN_STREAM(
        node_->get_logger(),
        "image frame " << data->t_ns << " dropped due to queue overflow");
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
}
}  // namespace basalt_ros
