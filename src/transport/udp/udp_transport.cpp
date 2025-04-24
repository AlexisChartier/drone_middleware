
#include "dmw/transport/itransport.hpp"
#include "dmw/transport/factory.hpp"
#include <asio.hpp>
#include <iostream>

namespace dmw::transport
{
class UdpTransport : public ITransport
{
public:
    UdpTransport() : socket_{io_} {}

    bool connect(std::string_view host, uint16_t port) override
    {
        try {
            asio::ip::udp::resolver res{io_};
            auto endpoints = res.resolve(
                asio::ip::udp::v4(), host, std::to_string(port));
            endpoint_ = *endpoints.begin();
            socket_.open(asio::ip::udp::v4());
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[UDP] connect error: " << e.what() << '\n';
            return false;
        }
    }

    bool send(PacketView p) noexcept override
    {
        std::error_code ec;
        socket_.send_to(asio::const_buffer{p.data, p.size}, endpoint_, 0, ec);
        return !ec;
    }

    void close() noexcept override
    {
        std::error_code ec;
        socket_.close(ec);
    }
private:
    asio::io_context          io_;
    asio::ip::udp::socket     socket_;
    asio::ip::udp::endpoint   endpoint_;
};

/* --- auto-enregistrement dans la Factory --- */
static bool reg_udp = []{
    Factory::instance().register_backend(
        "udp", []{ return std::make_unique<UdpTransport>(); });
    return true;
}();

} // namespace dmw::transport
