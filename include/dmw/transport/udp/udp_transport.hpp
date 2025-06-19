#pragma once
#include <asio.hpp>
#include <string>
#include <memory>
#include "dmw/transport/itransport.hpp"

namespace dmw::transport::udp
{
using asio::ip::udp;

/** Transport UDP très simple : envoi de datagrammes vers une adresse/port. */
class UdpTransport : public dmw::transport::ITransport
{
public:
    UdpTransport();
    bool connect(std::string_view host, uint16_t port) override;  // ✅ Signature corrigée
    bool send(PacketView p) noexcept override;
    void close() noexcept override;

private:
    asio::io_context io_;
    udp::socket      sock_;
    udp::endpoint    dst_;
};

} // namespace dmw::transport::udp
