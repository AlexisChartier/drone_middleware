/**********************************************************************
 *  OctomapSub  –  écoute un topic OctoMap binaire et envoie
 *                 des fragments via ITransport.
 *********************************************************************/
#include "dmw/ros/octomap_sub.hpp"

#include <octomap_msgs/conversions.h>
#include <zlib.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace dmw::ros
{
// ===================================================================
// utilitaires
// ===================================================================

// -- zlib ----------------------------------------------------------------
static std::vector<uint8_t> compress_vec(const std::vector<int8_t>& in)
{
  uLongf dst = compressBound(in.size());
  std::vector<uint8_t> out(dst);

  if (::compress2(out.data(), &dst,
                  reinterpret_cast<const Bytef*>(in.data()),
                  in.size(), 1) != Z_OK)
    throw std::runtime_error("zlib compress failed");

  out.resize(dst);
  return out;
}

// -- helper param() : déclare si besoin puis lit -------------------------
template<typename T>
T param(rclcpp::Node* n, const std::string& name, const T& def_val)
{
  if (!n->has_parameter(name))
    n->declare_parameter<T>(name, def_val);
  return n->get_parameter(name).get_value<T>();
}

// ===================================================================
// Ctor
// ===================================================================
OctomapSub::OctomapSub(std::shared_ptr<dmw::transport::ITransport> tx)
: Node("octomap_relay_node",
       rclcpp::NodeOptions{}
         .allow_undeclared_parameters(true)
         .automatically_declare_parameters_from_overrides(true)),
  tx_{std::move(tx)},
  builder_{1300, 0}
{
  snapshot_interval_ = param(this, "snapshot_interval_s", 10.0);
  compress_          = param(this, "compress",            true);

  std::string topic  = param(this, "octomap_topic",
                             std::string("octomap_binary"));   // ← relatif

  if (!topic.empty() && topic.front() != '/')
    topic = std::string(get_namespace()) + "/" + topic;

  // QoS identique à octomap_server (Reliable + TransientLocal, depth 1)
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability (rclcpp::DurabilityPolicy::TransientLocal);

  sub_ = create_subscription<octomap_msgs::msg::Octomap>(
           topic, qos,
           std::bind(&OctomapSub::callback, this, std::placeholders::_1));

  last_snapshot_ = now();
}

// ===================================================================
// callback
// ===================================================================
void OctomapSub::callback(octomap_msgs::msg::Octomap::SharedPtr msg)
{
  const auto now_ts = now();

  auto send_blob = [&](const std::vector<uint8_t>& blob, uint8_t flags)
  {
    auto sendFn = [&](auto&& bufs)
    {
      dmw::transport::PacketView view{bufs.data(), bufs.size()};
      tx_->send(view);
    };
    builder_.build_and_send(blob.data(),
                            static_cast<uint32_t>(blob.size()),
                            flags,
                            sendFn);
  };

  // ---- snapshot périodique ------------------------------------------
  if ((now_ts - last_snapshot_).seconds() >= snapshot_interval_ || !last_tree_)
  {
    std::vector<uint8_t> bytes(msg->data.begin(), msg->data.end());
    if (compress_)
      bytes = compress_vec(msg->data);

    send_blob(bytes, dmw::core::SNAPSHOT);
    last_snapshot_ = now_ts;
  }

  // ---- delta-update désactivé pour l’instant ------------------------

  last_tree_.reset(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
}

} // namespace dmw::ros
