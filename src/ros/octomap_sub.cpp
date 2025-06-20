/**********************************************************************
 *  OctomapSub  –  relaie un topic OctoMap binaire vers l’UDP
 *                 (format commun Drone ↔ Serveur).
 *********************************************************************/
#include "dmw/ros/octomap_sub.hpp"

#include "dmw/core/binary_packet_builder.hpp"   // builder (ptr + size)
#include "dmw/core/udp_header.hpp"              // même struct des deux côtés
#include <dmw/transport/itransport.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include <rclcpp/rclcpp.hpp>

#include <zlib.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace dmw::ros
{
// ===================================================================
// • utilitaires
// ===================================================================

// — compression zlib (niveau 1) -------------------------------------
[[nodiscard]]
static std::vector<uint8_t> zlib_compress(const std::vector<int8_t>& in)
{
  uLongf dst_sz = compressBound(in.size());
  std::vector<uint8_t> out(dst_sz);

  if (::compress2(out.data(), &dst_sz,
                  reinterpret_cast<const Bytef*>(in.data()),
                  in.size(), 1) != Z_OK)
    throw std::runtime_error("zlib compress failed");

  out.resize(dst_sz);
  return out;
}

// — helper param() : déclare si besoin puis lit ---------------------
template<typename T>
T param(rclcpp::Node* n, const std::string& name, const T& def_val)
{
  if (!n->has_parameter(name))
    n->declare_parameter<T>(name, def_val);
  return n->get_parameter(name).template get_value<T>();
}

static uint8_t numeric_id(const std::string & id)
{
  for (char c : id)
    if (std::isdigit(c))
      return static_cast<uint8_t>(std::atoi(&c));
  return 0;
}

// ===================================================================
// • constructeur
// ===================================================================
OctomapSub::OctomapSub(std::shared_ptr<dmw::transport::ITransport> tx)
: Node("octomap_relay_node",
       rclcpp::NodeOptions{}
         .allow_undeclared_parameters(true)
         .automatically_declare_parameters_from_overrides(true)),
  tx_{std::move(tx)}
  {
  /* --- paramètres ------------------------------------------------- */
  snapshot_interval_ = param(this, "snapshot_interval_s", 10.0);
  compress_          = param(this, "compress",            true);

  /* id numérique pour l’en-tête UDP --------------------------------- */
  std::string drone_id_str = param(this, "drone_id",
                                      std::string("DT1"));
  uint8_t     drone_num_id = numeric_id(drone_id_str);

  /* (ré)-initialise le BinaryPacketBuilder avec l’ID numérique ---- */
  builder_ = dmw::core::BinaryPacketBuilder(dmw::core::DEFAULT_MTU,
                                            drone_num_id);

  std::string topic  = param(this, "octomap_topic",
                             std::string("octomap_binary"));   // relatif

  if (!topic.empty() && topic.front() != '/')
    topic = std::string(get_namespace()) + "/" + topic;

  RCLCPP_INFO(get_logger(), "OctomapSub: subscribe → %s", topic.c_str());

  /* --- QoS identique à octomap_server ----------------------------- */
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability (rclcpp::DurabilityPolicy::TransientLocal);

  sub_ = create_subscription<octomap_msgs::msg::Octomap>(
           topic, qos,
           std::bind(&OctomapSub::callback, this, std::placeholders::_1));

  last_snapshot_ = now();
}

// ===================================================================
// • callback principal
// ===================================================================
void OctomapSub::callback(octomap_msgs::msg::Octomap::SharedPtr msg)
{
  const auto now_ts = now();

  /* --- helper : expédie le blob via le builder -------------------- */
  auto send_blob = [&](const std::vector<uint8_t>& blob, uint8_t flags)
  {
    builder_.build_and_send(
        blob.data(),
        static_cast<uint32_t>(blob.size()),
        flags,
        /* SendFn : ptr + size -------------------------------------- */
        [this](const uint8_t* ptr, std::size_t n)
        {
          dmw::transport::PacketView v{ptr, n};
          tx_->send(v);
        });
  };

  /* --- snapshot périodique ---------------------------------------- */
  if ((now_ts - last_snapshot_).seconds() >= snapshot_interval_ || !last_tree_)
  {
    std::vector<uint8_t> payload;
    uint8_t flags = dmw::core::FLAG_SNAPSHOT;

    if (compress_) {
      std::vector<uint8_t> cmp = zlib_compress(msg->data);
      uint32_t raw_sz = static_cast<uint32_t>(msg->data.size());

      payload.resize(4 + cmp.size());
      std::memcpy(payload.data(), &raw_sz, 4);
      std::memcpy(payload.data() + 4, cmp.data(), cmp.size());

      flags |= dmw::core::FLAG_GZIP;
    } else {
      payload.assign(msg->data.begin(), msg->data.end());
    }

    send_blob(payload, flags);
    last_snapshot_ = now_ts;
  }

  /* --- delta-update non implémenté (placeholder) ------------------ */

  last_tree_.reset(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
}

} // namespace dmw::ros
