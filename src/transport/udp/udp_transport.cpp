#include "dmw/transport/udp/udp_transport.hpp"

namespace dmw::transport::udp
{
using asio::ip::udp;

UdpTransport::UdpTransport()
  : sock_(io_)
{}

bool UdpTransport::connect(const std::string& host, uint16_t port)
{
    try {
        sock_.open(udp::v4());
        sock_.bind(udp::endpoint(udp::v4(), 0));  // port éphémère
        asio::ip::address addr = asio::ip::make_address(host);
        dst_ = udp::endpoint(addr, port);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

} // namespace dmw::transport::udp
