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
#include <hydra_stretch_msgs/HydraVisionPacket.h>
#include <hydra_stretch_msgs/Masks.h>
#include <sensor_msgs/Image.h>

#include <boost/smart_ptr/make_shared_object.hpp>

namespace hydra {

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
  field(config.skip_frame, "skip_frame");
}

ImageReceiver::ImageReceiver(const Config& config, size_t sensor_id)
    : DataReceiver(config, sensor_id), config(config), nh_(config.ns) {}

bool ImageReceiver::initImpl() {
  vision_packet_sub_ = nh_.subscribe<hydra_stretch_msgs::HydraVisionPacket>(
      "vision_packet", config.queue_size, &ImageReceiver::callback, this);
  return true;
}

ImageReceiver::~ImageReceiver() {}

std::string showImageDim(const sensor_msgs::Image::ConstPtr& image) {
  std::stringstream ss;
  ss << "[" << image->width << ", " << image->height << "]";
  return ss.str();
}

std::string showMaskDim(const hydra_stretch_msgs::Masks::ConstPtr& masks) {
  std::stringstream ss;
  ss << "[" << masks->masks[0].data.width << ", " << masks->masks[0].data.height << "]";
  return ss.str();
}

void ImageReceiver::callback(
    const hydra_stretch_msgs::HydraVisionPacket::ConstPtr& vision_packet_msg) {
  if (frame_cnt++ > config.skip_frame) {
    const auto color_msg =
        boost::make_shared<sensor_msgs::Image>(vision_packet_msg->color);

    const auto depth_msg =
        boost::make_shared<sensor_msgs::Image>(vision_packet_msg->depth);

    const auto labels_msg =
        boost::make_shared<sensor_msgs::Image>(vision_packet_msg->label);

    const auto masks_msg =
        boost::make_shared<hydra_stretch_msgs::Masks>(vision_packet_msg->masks);

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

    if (masks_msg->masks.size() > 0 &&
        (masks_msg->masks[0].data.width != depth_msg->width ||
         masks_msg->masks[0].data.height != depth_msg->height)) {
      LOG(ERROR) << "masks dimensions do not match depth dimensions: "
                 << showMaskDim(masks_msg) << " != " << showImageDim(depth_msg);
      return;
    }

    if (!checkInputTimestamp(depth_msg->header.stamp.toNSec())) {
      return;
    }

    auto packet = std::make_shared<ImageInputPacket>(color_msg->header.stamp.toNSec(),
                                                     sensor_id_);
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
          MaskData mask_data;
          mask_data.class_id = mask_msg.class_id;
          mask_data.mask = cv_mask->image;
          // NOTE: Check this part if something goes wrong (turn verbosity to 2),
          // sometimes the class id could not be received
          VLOG(2) << "[ImageReceiver] Received mask data with class id: "
                  << mask_msg.class_id;
          VLOG(2) << "[ImageReceiver] Registering mask data with class id: "
                  << mask_data.class_id;
          VLOG(2) << "[ImageReceiver] Received mask data with mask size w: "
                  << cv_mask->image.cols << " h: " << cv_mask->image.rows;
          VLOG(2) << "[ImageReceiver] Registering mask data with mask size w: "
                  << mask_data.mask.size().width << " h: " << mask_data.mask.rows;
          packet->instance_masks.push_back(mask_data);
        }
      }
    } catch (const cv_bridge::Exception& e) {
      LOG(ERROR) << "unable to read images from ros: " << e.what();
    }
    queue.push(packet);
  } else {
    LOG(INFO) << "Skipping frame " << frame_cnt;
  }
}

}  // namespace hydra
