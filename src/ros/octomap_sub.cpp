#include "dmw/ros/octomap_sub.hpp"

#include <zlib.h>
#include <algorithm>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace dmw::ros
{
/* ------------ helper compression (zlib niveau 1) ---------------- */
static std::vector<uint8_t> compress_vec(const std::vector<int8_t>& in)
{
  uLongf dst_sz = compressBound(in.size());
  std::vector<uint8_t> out(dst_sz);

  if (compress2(out.data(), &dst_sz,
                reinterpret_cast<const Bytef*>(in.data()),
                in.size(), 1) != Z_OK)
    throw std::runtime_error("zlib compress failed");

  out.resize(dst_sz);
  return out;
}

/* ---------------------- constructeur ----------------------------- */
OctomapSub::OctomapSub(std::shared_ptr<dmw::transport::ITransport> tx)
  : Node("octomap_relay_node"),                 //
    tx_{std::move(tx)},                         //
    builder_{1300, 0}                           // MTU 1300, channel 0
{
  /* paramètres avec valeurs par défaut */
  const double snapshot_def = 10.0;
  snapshot_interval_ =
      declare_parameter<double>("snapshot_interval_s", snapshot_def);
  compress_ =
      declare_parameter<bool>("compress", true);

  std::string topic =
      declare_parameter<std::string>("octomap_topic", "octomap_binary"); // ← RELATIF

  /* si l’utilisateur n’a pas mis de « / » on préfixe par le namespace
     du nœud (ex :  "/DT1/octomap_binary")                              */
  if (!topic.empty() && topic.front() != '/')
    topic = get_namespace() + std::string("/") + topic;

  /* QoS identique à octomap_server  (Reliable + TransientLocal, depth 1) */
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability (rclcpp::DurabilityPolicy::TransientLocal);

  sub_ = create_subscription<octomap_msgs::msg::Octomap>(
           topic, qos,
           std::bind(&OctomapSub::callback, this, std::placeholders::_1));

  last_snapshot_ = now();
}

/* ----------------------------- callback -------------------------- */
void OctomapSub::callback(octomap_msgs::msg::Octomap::SharedPtr msg)
{
  const auto now_ts = now();

  /* — envoi d’un blob via le transport — */
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

  /* (1) snapshot complet périodique -------------------------------- */
  if ((now_ts - last_snapshot_).seconds() >= snapshot_interval_ || !last_tree_)
  {
    std::vector<uint8_t> bytes(msg->data.begin(), msg->data.end());

    if (compress_)
      bytes = compress_vec(msg->data);

    send_blob(bytes, dmw::core::SNAPSHOT);
    last_snapshot_ = now_ts;
  }

  /* (2) delta-update désactivé pour l’instant ---------------------- */

  last_tree_.reset(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
}
} // namespace dmw::ros
