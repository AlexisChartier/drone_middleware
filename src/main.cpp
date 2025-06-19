// ────────────────────────────────────────────────────────────────
//  Drone-Middleware – main.cpp   (Octomap → UDP fragments)
// ────────────────────────────────────────────────────────────────
#include "dmw/ros/octomap_sub.hpp"
#include "dmw/transport/factory.hpp"
#include "dmw/transport/url.hpp"
#include "dmw/net/net_guard.hpp"               //  ← NEW

#include <rclcpp/rclcpp.hpp>

#include <cctype>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <cstring>

/* helper ─────────────────────────────────────────────────────── */
static uint8_t numeric_id(const std::string & id)
{
  for (char c : id)
    if (std::isdigit(c))
      return static_cast<uint8_t>(std::atoi(&c));
  return 0;
}

/* ============================================================= *
 *          Buffered transport protected by a NetGuard           *
 * ============================================================= */
class GuardedTx : public dmw::transport::ITransport
{
public:
  GuardedTx(std::unique_ptr<dmw::transport::ITransport> impl,
            dmw::net::NetGuard                       * guard,
            bool                                       enabled)
  : impl_{std::move(impl)}, guard_{guard}, enabled_{enabled} {}

  /* real connect is forwarded unchanged ------------------------ */
  bool connect(std::string_view h, uint16_t p) override
  { return impl_->connect(h, p); }

  void close() noexcept override { impl_->close(); }

  bool send(dmw::transport::PacketView v) noexcept override
  {
    if (enabled_ && !guard_->ready())
    {   /* bad link → stash a copy                               */
      offline_.emplace_back(
        static_cast<const uint8_t*>(v.data),
        static_cast<const uint8_t*>(v.data) + v.size);
      return true;
    }
    flush();
    return impl_->send(v);
  }

  void flush()
  {
    while (!offline_.empty())
    {
      const auto & b = offline_.front();
      impl_->send({b.data(), b.size()});
      offline_.pop_front();
    }
  }

private:
  std::unique_ptr<dmw::transport::ITransport> impl_;
  dmw::net::NetGuard                       * guard_;
  const bool                                 enabled_;
  std::deque<std::vector<uint8_t>>           offline_;
};

/* =============================  main  ========================= */
int main(int argc,char** argv)
{
  /* 1 ─ read parameters with a throw-away node ----------------- */
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);
  auto pnode = std::make_shared<rclcpp::Node>("_params",opts);

  const std::string drone_id   = pnode->get_parameter_or("drone_id",   std::string("DT1"));
  const std::string url_str    = pnode->get_parameter_or("transport_url",std::string("udp://127.0.0.1:48484"));
  const int         mtu        = pnode->get_parameter_or("mtu_payload",1300);
  const bool        compress   = pnode->get_parameter_or("compress",   true);
  std::string       topic      = pnode->get_parameter_or("octomap_topic",std::string("/octomap_binary"));

  const bool guard_on  = pnode->get_parameter_or("enable_net_guard",true);
  const std::string wifi_if = pnode->get_parameter_or("wifi_iface",std::string("wlan0"));
  const int  rssi_min  = pnode->get_parameter_or("rssi_min",-75);

  if (!topic.empty() && topic.front()!='/')
    topic = std::string(pnode->get_namespace()) + "/" + topic;

  /* 2 ─ build real transport + NetGuard ------------------------ */
  auto url    = dmw::transport::parse(url_str);
  auto raw_tx = dmw::transport::Factory::instance().make(url);
  if (!raw_tx || !raw_tx->connect(url.host,url.port))
  {
    RCLCPP_FATAL(pnode->get_logger(),"Cannot create transport for %s",url_str.c_str());
    return 1;
  }

  dmw::net::NetGuard guard{*pnode,wifi_if,url.host,rssi_min};
  auto tx = std::make_shared<GuardedTx>(std::move(raw_tx),&guard,guard_on);

  RCLCPP_INFO(pnode->get_logger(),
      "Drone %s(id=%u) → %s:%u  | topic=%s | guard=%s(if=%s,rssi>%d)",
      drone_id.c_str(),numeric_id(drone_id),
      url.host.c_str(),url.port,topic.c_str(),
      guard_on?"on":"off",wifi_if.c_str(),rssi_min);

  /* 3 ─ spawn OctomapSub (topic + fragmenter) ------------------ */
  auto octo = std::make_shared<dmw::ros::OctomapSub>(tx);
  octo->set_parameters({
      rclcpp::Parameter("octomap_topic",topic),
      rclcpp::Parameter("compress",compress),
      rclcpp::Parameter("mtu_payload",mtu)
  });

  /* 4 ─ run ---------------------------------------------------- */
  rclcpp::spin(octo);
  tx->close();
  rclcpp::shutdown();
  return 0;
}
