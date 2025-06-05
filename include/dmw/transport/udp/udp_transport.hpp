#pragma once
#include <asio.hpp>
#include <string>
#include <memory>

namespace dmw::transport::udp
{
using asio::ip::udp;

/** Transport UDP très simple : envoi de datagrammes vers une adresse/port. */
class UdpTransport : public std::enable_shared_from_this<UdpTransport>
{
public:
    UdpTransport();  // constructeur par défaut
    bool connect(const std::string& host, uint16_t port);  // configuration post-creation

    void send(const void* data, std::size_t len)
    {
        sock_.send_to(asio::buffer(data, len), dst_);
    }

private:
    asio::io_context io_;
    udp::socket      sock_;
    udp::endpoint    dst_;
};

} // namespace dmw::transport::udp
    