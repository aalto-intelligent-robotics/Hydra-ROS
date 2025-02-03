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
#include "hydra_ros/utils/dsg_streaming_interface.h"

#include <Eigen/src/Core/Matrix.h>
#include <glog/logging.h>
#include <hydra/common/dsg_types.h>
#include <hydra/utils/display_utilities.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <hydra/utils/timing_utilities.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo_ros/conversion/ros_conversion.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

namespace hydra {

DsgSender::DsgSender(const ros::NodeHandle& nh,
                     const std::string& frame_id,
                     const std::string& timer_name,
                     bool publish_mesh,
                     double min_mesh_separation_s,
                     bool serialize_dsg_mesh)
    : nh_(nh),
      frame_id_(frame_id),
      timer_name_(timer_name),
      publish_mesh_(publish_mesh),
      min_mesh_separation_s_(min_mesh_separation_s),
      serialize_dsg_mesh_(serialize_dsg_mesh) {
  pub_ = nh_.advertise<hydra_msgs::DsgUpdate>("dsg", 1);
  object_pub_ = nh_.advertise<hydra_stretch_msgs::ObjectLayerInfo>("object_nodes", 1);
  if (publish_mesh_) {
    mesh_pub_ = nh_.advertise<kimera_pgmo_msgs::KimeraPgmoMesh>("dsg_mesh", 1, false);
  }
}

void DsgSender::sendGraph(const DynamicSceneGraph& graph,
                          const ros::Time& stamp) const {
  const uint64_t timestamp_ns = stamp.toNSec();
  timing::ScopedTimer timer(timer_name_, timestamp_ns);

  if (pub_.getNumSubscribers()) {
    hydra_msgs::DsgUpdate msg;
    msg.header.stamp = stamp;
    spark_dsg::io::binary::writeGraph(graph, msg.layer_contents, serialize_dsg_mesh_);
    msg.full_update = true;
    pub_.publish(msg);
  }

  //! TEST: Publish node information
  hydra_stretch_msgs::ObjectLayerInfo all_objects_nodes_msg;
  for (const auto& node : graph.getLayer(DsgLayers::OBJECTS).nodes()) {
    hydra_stretch_msgs::ObjectNodeInfo object_node_msg;

    object_node_msg.node_id = node.first;

    const ObjectNodeAttributes& node_attrs =
        node.second->attributes<ObjectNodeAttributes>();

    const Eigen::Vector3f& bbox_dims = node_attrs.bounding_box.dimensions;
    const Eigen::Vector3f& world_P_center = node_attrs.bounding_box.world_P_center;
    object_node_msg.name = node_attrs.name;
    object_node_msg.bounding_box.dimensions = {
        bbox_dims.x(), bbox_dims.y(), bbox_dims.z()};
    object_node_msg.bounding_box.world_P_center = {
        world_P_center.x(), world_P_center.y(), world_P_center.z()};
    object_node_msg.class_id = node_attrs.semantic_label;
    object_node_msg.position = {node_attrs.position.x(), node_attrs.position.y(), node_attrs.position.z()};
    hydra_stretch_msgs::InstanceViewHeader instance_view_header;
    for (const auto& view : node_attrs.instance_views.id_to_instance_masks_) {
      instance_view_header.map_view_id = view.first;
      instance_view_header.mask_id = view.second.mask_id_;
      object_node_msg.instance_view_headers.push_back(instance_view_header);
    }

    all_objects_nodes_msg.nodes.push_back(object_node_msg);
  }
  object_pub_.publish(all_objects_nodes_msg);

  if (!publish_mesh_ || !mesh_pub_.getNumSubscribers()) {
    return;
  }

  auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  if (last_mesh_time_ns_) {
    std::chrono::nanoseconds diff_ns(timestamp_ns - *last_mesh_time_ns_);
    std::chrono::duration<double> diff_s = diff_ns;
    if (diff_s.count() < min_mesh_separation_s_) {
      return;
    }
  }

  last_mesh_time_ns_ = timestamp_ns;

  kimera_pgmo_msgs::KimeraPgmoMesh msg = kimera_pgmo::conversions::toMsg(*mesh);
  msg.header.stamp.fromNSec(timestamp_ns);
  msg.header.frame_id = frame_id_;
  mesh_pub_.publish(msg);
}

DsgReceiver::DsgReceiver(const ros::NodeHandle& nh, bool subscribe_to_mesh)
    : nh_(nh), has_update_(false), graph_(nullptr) {
  sub_ = nh_.subscribe("dsg", 1, &DsgReceiver::handleUpdate, this);
  if (subscribe_to_mesh) {
    mesh_sub_ = nh_.subscribe("dsg_mesh_updates", 1, &DsgReceiver::handleMesh, this);
  }
}

DsgReceiver::DsgReceiver(const ros::NodeHandle& nh, const LogCallback& log_cb)
    : DsgReceiver(nh) {
  log_callback_.reset(new LogCallback(log_cb));
}

void DsgReceiver::handleUpdate(const hydra_msgs::DsgUpdate::ConstPtr& msg) {
  timing::ScopedTimer timer("receive_dsg", msg->header.stamp.toNSec());
  if (!msg->full_update) {
    throw std::runtime_error("not implemented");
  }

  if (log_callback_) {
    (*log_callback_)(msg->header.stamp, msg->layer_contents.size());
  }

  const auto size_bytes = getHumanReadableMemoryString(msg->layer_contents.size());
  VLOG(5) << "Received dsg update message of " << size_bytes;
  try {
    if (!graph_) {
      graph_ = spark_dsg::io::binary::readGraph(msg->layer_contents);
    } else {
      spark_dsg::io::binary::updateGraph(*graph_, msg->layer_contents);
    }
    has_update_ = true;
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Received invalid message: " << e.what());
    ros::shutdown();
    return;
  }

  if (mesh_) {
    graph_->setMesh(mesh_);
  }
}

void DsgReceiver::handleMesh(const kimera_pgmo_msgs::KimeraPgmoMesh::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  timing::ScopedTimer timer("receive_mesh", msg->header.stamp.toNSec());
  if (!mesh_) {
    mesh_ = std::make_shared<Mesh>();
  }

  kimera_pgmo::conversions::fromMsg(*msg, *mesh_);

  if (graph_) {
    graph_->setMesh(mesh_);
  }

  has_update_ = true;
}

}  // namespace hydra
