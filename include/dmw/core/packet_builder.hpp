#pragma once
#include <vector>
#include <cstdint>
#include "voxel.hpp"

namespace dmw::core
{
struct PacketHeader
{
    uint8_t  ver {1};
    uint8_t  flags {0};
    uint16_t drone_id {0};
    uint16_t count {0};      ///< # voxels dans payload
    uint16_t reserved {0};
};
static_assert(sizeof(PacketHeader)==8);

class PacketBuilder
{
public:
    explicit PacketBuilder(std::size_t mtu=1400)
        : mtu_{mtu} {}

    void reset(uint16_t drone_id) { hdr_.drone_id = drone_id; buf_.clear(); }

    bool add(const Voxel& v)
    {
        constexpr std::size_t voxelSize = sizeof(Voxel);
        if (buf_.size() + voxelSize > payload_capacity()) return false;
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
        buf_.insert(buf_.end(), p, p + voxelSize);
        return true;
    }

    /** Renvoie un tableau prêt à l'envoi et remet le builder à zéro. */
    std::vector<uint8_t> finalize()
    {
        hdr_.count = buf_.size() / sizeof(Voxel);
        std::vector<uint8_t> out(sizeof(PacketHeader));
        std::memcpy(out.data(), &hdr_, sizeof(PacketHeader));
        out.insert(out.end(), buf_.begin(), buf_.end());
        buf_.clear();
        return out;
    }
private:
    std::size_t payload_capacity() const { return mtu_ - sizeof(PacketHeader); }

    std::size_t mtu_;
    PacketHeader hdr_;
    std::vector<uint8_t> buf_;
};
} // namespace dmw::core
