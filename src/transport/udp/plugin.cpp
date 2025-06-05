#include "dmw/transport/factory.hpp"
#include "dmw/transport/udp/udp_transport.hpp"

namespace {
struct RegisterUdpTransport
{
    RegisterUdpTransport()
    {
        dmw::transport::Factory::instance().register_backend("udp", []() -> dmw::transport::Ptr {
            return std::make_shared<dmw::transport::udp::UdpTransport>();
        });
    }
};

static RegisterUdpTransport _reg_udp;
}
