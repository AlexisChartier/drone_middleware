#include "dmw/transport/udp/udp_transport.hpp"
#include <iostream>
#include <asio.hpp>

namespace dmw::transport::udp
{
using asio::ip::udp;

UdpTransport::UdpTransport()
  : sock_(io_)
{}

bool UdpTransport::connect(std::string_view host, uint16_t port)
{
    try {
        sock_.open(udp::v4());
        sock_.bind(udp::endpoint(udp::v4(), 0));  // port éphémère

        udp::resolver resolver(io_);
        udp::resolver::results_type endpoints =
            resolver.resolve(std::string(host), std::to_string(port));

        dst_ = *endpoints.begin();  // prend le premier résultat

        return true;
    } catch (const std::exception& e) {
        std::cerr << "UDP connect() failed: " << e.what() << std::endl;
        return false;
    }
}

bool UdpTransport::send(PacketView p) noexcept
{
    try {
        sock_.send_to(asio::buffer(p.data, p.size), dst_);
        return true;
    } catch (...) {
        return false;
    }
}

void UdpTransport::close() noexcept
{
    std::cout << "Closing UDP connection" << std::endl;
    sock_.close();
}

} // namespace dmw::transport::udp
