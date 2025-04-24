#pragma once
#include "dmw/transport/itransport.hpp"
#include <asio.hpp>

namespace dmw::transport
{
/** Back-end “UDP datagram” – implémente ITransport. */
class UdpTransport final : public ITransport
{
public:
    UdpTransport();

    /** Ouvre la socket et prépare l’endpoint (IP + port). */
    bool connect(std::string_view host, uint16_t port) override;

    /** Envoie <data,size> en un seul datagramme (best effort). */
    bool send(PacketView p) noexcept override;

    /** Ferme proprement la socket. */
    void close() noexcept override;

private:
    asio::io_context        io_;
    asio::ip::udp::socket   socket_;
    asio::ip::udp::endpoint endpoint_;
};
} // namespace dmw::transport
    