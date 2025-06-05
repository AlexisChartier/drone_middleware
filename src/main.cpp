// ────────────────────────────────────────────────────────────────
//  Drone Middleware – main.cpp   (Octomap binaire → UDP fragments)
// ────────────────────────────────────────────────────────────────
#include "dmw/transport/factory.hpp"
#include "dmw/transport/url.hpp"

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <zlib.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>
#include "dmw/transport/udp/plugin.cpp"  // force linkage du backend UDP

namespace dmw::core
{
#pragma pack(push, 1)
struct UdpHdr            // 16 octets
{
    uint32_t seq;        // identifiant du scan complet
    uint32_t tot;        // taille totale du blob
    uint32_t off;        // offset du fragment
    uint16_t len;        // longueur du fragment
    uint8_t  flags;      // 1 = snapshot, 2 = delta
    uint8_t  drone_id;   // 0‑255
};
#pragma pack(pop)

inline constexpr uint16_t DEFAULT_MTU   = 1300;   // payload ≤ 1300 o
inline constexpr uint8_t  FLAG_SNAPSHOT = 0x01;
inline constexpr uint8_t  FLAG_DELTA    = 0x02;   // réservé (delta‑update)
} // namespace dmw::core

/*──────────────────────── Programme ────────────────────────*/
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dmw_udp_octomap_relay");

    /* ---------- paramètres ROS ---------- */
    node->declare_parameter("drone_id",        1);
    node->declare_parameter("transport_url",   std::string("udp://127.0.0.1:48484"));
    node->declare_parameter("mtu_payload",     static_cast<int>(dmw::core::DEFAULT_MTU));
    node->declare_parameter("compress",        true);

    const uint16_t    drone_id   = static_cast<uint16_t>(node->get_parameter("drone_id").as_int());
    const std::string url_str    = node->get_parameter("transport_url").as_string();
    const uint16_t    mtu        = static_cast<uint16_t>(node->get_parameter("mtu_payload").as_int());
    const bool        compress   = node->get_parameter("compress").as_bool();

    /* ---------- transport UDP ---------- */
    auto url = dmw::transport::parse(url_str);
    auto tx  = dmw::transport::Factory::instance().make(url);
    if (!tx || !tx->connect(url.host, url.port))
    {
        RCLCPP_FATAL(node->get_logger(), "Cannot create transport for %s", url_str.c_str());
        return 1;
    }
    RCLCPP_INFO(node->get_logger(),
                "Drone %u → %s:%u  (MTU %u, compress %s)",
                drone_id, url.host.c_str(), url.port, mtu, compress ? "yes" : "no");

    /* ---------- abonne‑ment Octomap binaire ---------- */
    static std::atomic<uint32_t> seq_counter{0};

    node->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary",
        rclcpp::SensorDataQoS(),
        [&, mtu, compress, drone_id](const octomap_msgs::msg::Octomap::SharedPtr msg)
        {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(msg->data.data());
            uint32_t       size = static_cast<uint32_t>(msg->data.size());

            /* — compression optionnelle (zlib niveau 1) — */
            std::vector<uint8_t> zbuf;
            if (compress)
            {
                uLongf dst_len = compressBound(size);
                zbuf.resize(dst_len);
                if ( ::compress2(zbuf.data(), &dst_len, data, size, 1) == Z_OK )
                {
                    zbuf.resize(dst_len);
                    data = zbuf.data();
                    size = static_cast<uint32_t>(zbuf.size());
                }
            }

            const uint32_t seq = seq_counter.fetch_add(1, std::memory_order_relaxed);

            /* — fragmentation MTU‑aware — */
            for (uint32_t off = 0; off < size; off += mtu)
            {
                const uint16_t len = static_cast<uint16_t>(std::min<uint32_t>(mtu, size - off));

                dmw::core::UdpHdr hdr{
                    seq,            // identifiant
                    size,           // taille totale
                    off,            // offset
                    len,            // longueur fragment
                    dmw::core::FLAG_SNAPSHOT,
                    static_cast<uint8_t>(drone_id)
                };

                std::vector<uint8_t> packet(sizeof(hdr) + len);
                std::memcpy(packet.data(), &hdr, sizeof(hdr));
                std::memcpy(packet.data() + sizeof(hdr), data + off, len);

                tx->send({packet.data(), packet.size()});
            }
        });

    rclcpp::spin(node);
    tx->close();
    rclcpp::shutdown();
    return 0;
}
