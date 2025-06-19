// ────────────────────────────────────────────────────────────────
//  Drone Middleware – main.cpp   (Octomap binaire ⇒ UDP fragments)
// ----------------------------------------------------------------
#include "dmw/ros/octomap_sub.hpp"
#include "dmw/transport/factory.hpp"
#include "dmw/transport/url.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <string>

/* helper : extrait l’entier de “DT1”  →  1 (0-255) */
static uint8_t numeric_id(const std::string & id)
{
  for (char c : id)
    if (std::isdigit(c))
      return static_cast<uint8_t>(std::atoi(&c));
  return 0;
}

int main(int argc, char ** argv)
{
  /* ---------------------------------------------------------------- */
  /* 1) Initialisation ROS 2 + nœud temporaire pour lire les paramètres */
  /* ---------------------------------------------------------------- */
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);

  // nom vide ⇒ sera renommé depuis le launch-file
  auto param_node = std::make_shared<rclcpp::Node>("_params", opts);

    
    const std::string drone_id = param_node->get_parameter_or<std::string>("drone_id", "DT1");
    const std::string url_str  = param_node->get_parameter_or<std::string>(
                                    "transport_url", "udp://127.0.0.1:48484");
    const int  mtu       = param_node->get_parameter_or<int>("mtu_payload", 1300);
    const bool compress  = param_node->get_parameter_or<bool>("compress",   true);
    std::string topic    = param_node->get_parameter_or<std::string>("octomap_topic",
                                                                    "/octomap_binary");

  /* ---------------------------------------------------------------- */
  /* 2) Transport UDP                                                 */
  /* ---------------------------------------------------------------- */
  auto url = dmw::transport::parse(url_str);
  auto tx  = dmw::transport::Factory::instance().make(url);

  if (!tx || !tx->connect(url.host, url.port))
  {
    RCLCPP_FATAL(param_node->get_logger(),
                 "Cannot create transport for %s", url_str.c_str());
    return 1;
  }

  RCLCPP_INFO(param_node->get_logger(),
              "Drone %s (id=%u) → %s:%u",
              drone_id.c_str(), numeric_id(drone_id),
              url.host.c_str(), url.port);

  /* ---------------------------------------------------------------- */
  /* 3) Nœud applicatif : OctomapSub                                  */
  /* ---------------------------------------------------------------- */
  auto octo_node = std::make_shared<dmw::ros::OctomapSub>(std::move(tx));

  /* ---------------------------------------------------------------- */
  /* 4) Exécution                                                     */
  /* ---------------------------------------------------------------- */
  rclcpp::spin(octo_node);
  rclcpp::shutdown();
  return 0;
}
