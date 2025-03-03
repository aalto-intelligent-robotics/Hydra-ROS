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
#pragma once
#include <config_utilities/factory.h>
#include <hydra/input/data_receiver.h>
#include <hydra_msgs/HydraVisionPacket.h>
#include <hydra_msgs/Masks.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace hydra {

class ImageReceiver : public DataReceiver {
 public:
  struct Config : DataReceiver::Config {
    std::string ns = "~";
    size_t queue_size = 10;
    int skip_frame = 10;
  };

  ImageReceiver(const Config& config, size_t sensor_id);

  virtual ~ImageReceiver();

 public:
  const Config config;

 protected:
  bool initImpl() override;

 private:
  int frame_cnt = 0;
  /**
   * @brief callback function to extract information from HydraVisionPacket into
   * ImageInputPacket, which is then moved to Hydra for processing
   *
   * @param vision_packet_msg Custom ROS message which contains: {color, depth, labels,
   * masks}.
   */
  void callback(
      const hydra_msgs::HydraVisionPacket::ConstPtr& vision_packet_msg);

  ros::NodeHandle nh_;

  // Requires input to be sent as a whole package: {color, depth, label, masks}
  ros::Subscriber vision_packet_sub_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<DataReceiver,
                                     ImageReceiver,
                                     ImageReceiver::Config,
                                     size_t>("ImageReceiver");
};

void declare_config(ImageReceiver::Config& config);

}  // namespace hydra
