#pragma once
#include <asio.hpp>
#include <algorithm>
#include "udp_header.hpp"

namespace dmw::core {

class BinaryPacketBuilder {
public:
    explicit BinaryPacketBuilder(uint16_t mtu = 1300, uint8_t drone_id = 0)
        : mtu_{mtu}, drone_id_{drone_id} {}

    template<class SendFn>
    void build_and_send(const uint8_t* data, uint32_t size,
                        uint8_t flags, SendFn&& send)
    {
        static uint32_t seq = 0;
        for (uint32_t off = 0; off < size; off += mtu_) {
            uint16_t len = static_cast<uint16_t>(std::min<uint32_t>(mtu_, size - off));
            UdpHeader h{seq, size, off, len, flags, drone_id_};
            std::array<asio::const_buffer, 2> bufs{
                asio::buffer(&h, sizeof h),
                asio::buffer(data + off, len)
            };
            send(bufs);
        }
        ++seq;
    }
private:
    uint16_t mtu_;
    uint8_t  drone_id_;
};

} // namespace dmw::core