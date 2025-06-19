#pragma once
#include <atomic>
#include <string>
#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

namespace dmw::net {

class NetGuard
{
public:
  NetGuard(rclcpp::Node& n,
           std::string   iface,
           std::string   host,
           int           rssi_min)
  : nh_{n}, iface_{std::move(iface)}, host_{std::move(host)},
    rssi_min_{rssi_min}
  {
    /* every 5 s → update ok_ */
    timer_ = nh_.create_wall_timer(
      std::chrono::seconds(5),
      [this]() { ok_.store( link_good() ); });
  }

  bool ready() const { return ok_.load(); }

private:
  bool link_good()
  {
    return (read_rssi() >= rssi_min_) && ping_ok();
  }

  int read_rssi() const                 /* “iw … | grep signal” */
  {
    std::string cmd = "iw dev " + iface_ + " link | grep signal | awk '{print $2}'";
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return -999;
    int rssi{};
    fscanf(p, "%d", &rssi);
    pclose(p);
    return rssi;                        /* −55  (good) … −90 (bad) */
  }

  bool ping_ok() const
  {
    std::string cmd = "ping -c1 -W1 " + host_ + " >/dev/null 2>&1";
    return system(cmd.c_str()) == 0;
  }

  rclcpp::Node&                    nh_;
  std::string                      iface_, host_;
  const int                        rssi_min_;
  rclcpp::TimerBase::SharedPtr     timer_;
  std::atomic<bool>                ok_{false};
};

} // namespace dmw::net
