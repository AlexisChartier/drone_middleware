#include "dmw/core/voxel_buffer.hpp"
#include "dmw/core/packet_builder.hpp"
#include "dmw/ros/octomap_sub.hpp"
#include "dmw/transport/factory.hpp"
#include "dmw/transport/url.hpp"

#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace dmw;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // paramètres par défaut
    uint16_t drone_id      = 1;
    std::string url_str    = "udp://127.0.0.1:48484";
    int flush_ms           = 30;

    // Récupération des paramètres ROS
    rclcpp::NodeOptions opts;
    auto param_node = std::make_shared<rclcpp::Node>("dmw_param");
    param_node->declare_parameter("drone_id", drone_id);
    param_node->declare_parameter("transport_url", url_str);
    param_node->declare_parameter("flush_ms", flush_ms);
    drone_id   = param_node->get_parameter("drone_id").as_int();
    url_str    = param_node->get_parameter("transport_url").as_string();
    flush_ms   = param_node->get_parameter("flush_ms").as_int();

    transport::Url url = transport::parse(url_str);
    auto tx = transport::Factory::instance().make(url);
    if (!tx || !tx->connect(url.host, url.port))
    {
        RCLCPP_ERROR(param_node->get_logger(), "Cannot create transport");
        return 1;
    }

    core::VoxelBuffer<16384> buf;
    packet_builder::core::PacketBuilder builder(1400);
    builder.reset(drone_id);

    // Node ROS dédié à l'abonnement Octomap
    auto octo_sub = std::make_shared<ros::OctomapSub>(buf, drone_id);

    /** Thread réseau : flush périodique */
    std::thread net([&]{
        std::vector<core::Voxel> lot;
        lot.reserve(256);

        while (rclcpp::ok()) {
            lot.clear();
            buf.pop_bulk(lot);
            for (const auto& v : lot)
                if (!builder.add(v)) {
                    auto pkt = builder.finalize();
                    tx->send({pkt.data(), pkt.size()});
                    builder.reset(drone_id);
                    builder.add(v);            // retry sur lot suivant
                }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(flush_ms));
        }
        // envoyer le reste
        auto pkt = builder.finalize();
        if (!pkt.empty())
            tx->send({pkt.data(), pkt.size()});
    });

    rclcpp::spin(octo_sub);
    rclcpp::shutdown();
    net.join();
    tx->close();
    return 0;
}
