/* ─── core/binary_packet_builder.hpp ───────────────────────────── */
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>
#include "udp_header.hpp"

namespace dmw::core {

class BinaryPacketBuilder
{
public:
    BinaryPacketBuilder(uint16_t mtu = DEFAULT_MTU, uint8_t drone = 0)
      : mtu_{mtu}, drone_id_{drone} {}

    template<class SendFn>
    void build_and_send(const uint8_t* data,
                        uint32_t       size,
                        uint8_t        flags,
                        SendFn&&       send)
    {
        static uint32_t seq = 0;

        for (uint32_t off = 0; off < size; off += mtu_)
        {
            uint16_t len = static_cast<uint16_t>(
                              std::min<uint32_t>(mtu_, size - off));

            UdpHdr hdr{ seq, size, off, len, flags, drone_id_ };

            std::vector<uint8_t> packet(sizeof(UdpHdr) + len);
            std::memcpy(packet.data(),             &hdr, sizeof(UdpHdr));
            std::memcpy(packet.data() + sizeof(UdpHdr),
                        data + off, len);

            send(packet.data(), packet.size());          // ⬅ contiguous
        }
        ++seq;
    }

private:
    uint16_t mtu_;
    uint8_t  drone_id_;
};

} // namespace dmw::core
