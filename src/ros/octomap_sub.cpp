#include "dmw/ros/octomap_sub.hpp"

#include <zlib.h>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dmw::ros
{
/* ------------------------------------------------------------------ */
/*       Helper : compression simple (zlib niveau 1)                   */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/*                         Constructeur                                */
/* ------------------------------------------------------------------ */
OctomapSub::OctomapSub(std::shared_ptr<dmw::transport::ITransport> tx)
    : Node("octomap_relay_node"),
      tx_{std::move(tx)},
      builder_{1300, 0}          /* MTU 1300, channel 0 */
{
  /* paramètres ROS */
  declare_parameter("snapshot_interval_s", 10.0);
  declare_parameter("compress", true);
  snapshot_interval_ = get_parameter("snapshot_interval_s").as_double();
  compress_          = get_parameter("compress").as_bool();

  sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", rclcpp::SensorDataQoS(),
      std::bind(&OctomapSub::callback, this, std::placeholders::_1));

  last_snapshot_ = now();
}

/* ------------------------------------------------------------------ */
/*                           Callback                                  */
/* ------------------------------------------------------------------ */
void OctomapSub::callback(octomap_msgs::msg::Octomap::SharedPtr msg)
{
  const auto now_ts = now();

  /* helper : expédie un bloc via le transport */
  auto send_blob = [&](const std::vector<uint8_t>& blob, uint8_t flags)
  {
    /*  lambda intermédiaire : reçoit l’array<asio::const_buffer,2>           */
    auto sendFn = [&](auto&& bufs)
    {
      /* PacketView = {ptr,size} : on le construit à partir du tableau        */
      dmw::transport::PacketView view{bufs.data(), bufs.size()};
      tx_->send(view);
    };

    builder_.build_and_send(blob.data(),
                            static_cast<uint32_t>(blob.size()),
                            flags,
                            sendFn);
  };

  /* ---------------------------------------------------------------- */
  /*      (1) snapshot complet périodique                              */
  /* ---------------------------------------------------------------- */
  if ((now_ts - last_snapshot_).seconds() >= snapshot_interval_ || !last_tree_)
  {
    std::vector<uint8_t> bytes;
    bytes.reserve(msg->data.size());
    std::transform(msg->data.begin(), msg->data.end(),
                   std::back_inserter(bytes),
                   [](int8_t v){ return static_cast<uint8_t>(v); });

    if (compress_)
      bytes = compress_vec(msg->data);

    send_blob(bytes, dmw::core::SNAPSHOT);
    last_snapshot_ = now_ts;
  }

  /* ---------------------------------------------------------------- */
  /*      (2) delta ⇒ désactivé (API computeUpdate obsolète)          */
  /*          -> laisser pour portage ultérieur                       */
  /* ---------------------------------------------------------------- */

  /* mémorisation du dernier snapshot */
  last_tree_.reset(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
}

} // namespace dmw::ros
