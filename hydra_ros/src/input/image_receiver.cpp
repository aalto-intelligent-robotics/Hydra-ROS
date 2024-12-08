/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/input/image_receiver.h"

#include <config_utilities/config.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

namespace hydra {

using image_transport::ImageTransport;
using image_transport::SubscriberFilter;

image_transport::TransportHints getHintsWithNamespace(const ros::NodeHandle& nh,
                                                      const std::string& ns) {
  return image_transport::TransportHints(
      "raw", ros::TransportHints(), ros::NodeHandle(nh, ns));
}

void declare_config(ImageReceiver::Config& config) {
  using namespace config;
  name("ImageReceiver::Config");
  base<DataReceiver::Config>(config);
  field(config.ns, "ns");
  field(config.queue_size, "queue_size");
}

ImageSubscriber::ImageSubscriber() {}

ImageSubscriber::ImageSubscriber(const ros::NodeHandle& nh,
                                 const std::string& camera_name,
                                 const std::string& image_name,
                                 uint32_t queue_size)
    : transport(std::make_shared<ImageTransport>(ros::NodeHandle(nh, camera_name))),
      sub(std::make_shared<SubscriberFilter>(
          *transport, image_name, queue_size, getHintsWithNamespace(nh, camera_name))) {
}

ImageReceiver::ImageReceiver(const Config& config, size_t sensor_id)
    : DataReceiver(config, sensor_id), config(config), nh_(config.ns) {}

bool ImageReceiver::initImpl() {
  // TODO(nathan) subscribe to image subsets
  color_sub_ = ImageSubscriber(nh_, "rgb", "image_raw");
  depth_sub_ = ImageSubscriber(nh_, "depth_registered", "image_rect");
  label_sub_ = ImageSubscriber(nh_, "semantic");
  masks_sub_.subscribe(nh_, "masks", 1);
  synchronizer_.reset(new Synchronizer(SyncPolicy(config.queue_size),
                                       *color_sub_.sub,
                                       *depth_sub_.sub,
                                       *label_sub_.sub,
                                       masks_sub_));
  synchronizer_->registerCallback(&ImageReceiver::callback, this);
  return true;
}

ImageReceiver::~ImageReceiver() {}

std::string showImageDim(const sensor_msgs::Image::ConstPtr& image) {
  std::stringstream ss;
  ss << "[" << image->width << ", " << image->height << "]";
  return ss.str();
}

void ImageReceiver::callback(const sensor_msgs::Image::ConstPtr& color_msg,
                             const sensor_msgs::Image::ConstPtr& depth_msg,
                             const sensor_msgs::Image::ConstPtr& labels_msg,
                             const hydra_stretch_msgs::Masks::ConstPtr& masks_msg) {
  if (color_msg && (color_msg->width != depth_msg->width ||
                    color_msg->height != depth_msg->height)) {
    LOG(ERROR) << "color dimensions do not match depth dimensions: "
               << showImageDim(color_msg) << " != " << showImageDim(depth_msg);
    return;
  }

  if (labels_msg && (labels_msg->width != depth_msg->width ||
                     labels_msg->height != depth_msg->height)) {
    LOG(ERROR) << "label dimensions do not match depth dimensions: "
               << showImageDim(labels_msg) << " != " << showImageDim(depth_msg);
    return;
  }

  if (!checkInputTimestamp(depth_msg->header.stamp.toNSec())) {
    return;
  }

  auto packet = std::make_shared<ImageInputPacket>(
      color_msg->header.stamp.toNSec(), sensor_id_);
  try {
    const auto cv_depth = cv_bridge::toCvShare(depth_msg);
    packet->depth = cv_depth->image.clone();
    if (color_msg && color_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      auto cv_color = cv_bridge::toCvShare(color_msg);
      packet->color = cv_color->image.clone();
    } else if (color_msg) {
      auto cv_color =
          cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::RGB8);
      packet->color = cv_color->image;
    }

    if (labels_msg) {
      auto cv_labels = cv_bridge::toCvShare(labels_msg);
      packet->labels = cv_labels->image.clone();
    }

    if (masks_msg) {
      for (auto& mask_msg : masks_msg->masks) {
        auto cv_mask =
            cv_bridge::toCvCopy(mask_msg.data, sensor_msgs::image_encodings::MONO8);
        MaskData mask_data(mask_msg.class_id, cv_mask->image);
        packet->instance_masks.push_back(mask_data);
      }
    }
    else {
      LOG(WARNING) << "No Masks registered\n";
    }
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "unable to read images from ros: " << e.what();
  }

  queue.push(packet);
}

}  // namespace hydra
