#pragma once
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "dmw/core/voxel_buffer.hpp"
#include "dmw/core/morton.hpp"

namespace dmw::ros
{
class OctomapSub : public rclcpp::Node
{
public:
    OctomapSub(core::VoxelBuffer<16384>& buf, uint16_t drone_id);

private:
    void marker_callback(
        const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    core::VoxelBuffer<16384>& buf_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
    uint16_t drone_id_;
};
} // namespace dmw::ros
