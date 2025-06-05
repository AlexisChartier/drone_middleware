#pragma once
/* ─── ROS & OctoMap ──────────────────────────────────────────────── */
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

/* ─── STL ────────────────────────────────────────────────────────── */
#include <memory>
#include <vector>

/* ─── Middleware ─────────────────────────────────────────────────── */
#include "dmw/transport/itransport.hpp"
#include "dmw/core/binary_packet_builder.hpp"

namespace dmw::ros {

class OctomapSub : public rclcpp::Node
{
public:
  explicit OctomapSub(std::shared_ptr<dmw::transport::ITransport> tx);

private:
  void callback(octomap_msgs::msg::Octomap::SharedPtr msg);

  /* ─── state ─── */
  std::shared_ptr<dmw::transport::ITransport>                 tx_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  dmw::core::BinaryPacketBuilder                              builder_;

  double       snapshot_interval_{10.0};
  bool         compress_{true};
  rclcpp::Time last_snapshot_;
  std::shared_ptr<octomap::OcTree> last_tree_;
};

} // namespace dmw::ros
