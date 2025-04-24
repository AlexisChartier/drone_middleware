#include "dmw/ros/octomap_sub.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

using namespace dmw;

ros::OctomapSub::OctomapSub(core::VoxelBuffer<16384>& buf, uint16_t id)
: Node("octomap_voxel_streamer"),
  buf_{buf},
  drone_id_{id}
{
    sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/occupied_cells_vis_array", rclcpp::SystemDefaultsQoS(),
        std::bind(&OctomapSub::marker_callback, this, std::placeholders::_1));
}

void ros::OctomapSub::marker_callback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (const auto& mk : msg->markers) {
        for (const auto& p : mk.points) {
            uint32_t mx = static_cast<uint32_t>((p.x * 20.f) + 512); // 5 cm res
            uint32_t my = static_cast<uint32_t>((p.y * 20.f) + 512);
            uint32_t mz = static_cast<uint32_t>((p.z * 20.f) + 512);

            core::Voxel v;
            v.morton = core::encode_morton10(mx, my, mz);

            // simple gris par défaut (sera colorisé côté Octomap plus tard)
            v.rgb = 0x00AAAAAA;
            buf_.push(v);
        }
    }
}
