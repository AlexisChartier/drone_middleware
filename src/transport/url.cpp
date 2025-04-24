#include "dmw/transport/url.hpp"
#include <stdexcept>

namespace dmw::transport
{
Url parse(std::string_view s)
{
    auto pos_scheme = s.find("://");
    if (pos_scheme == std::string_view::npos)
        throw std::invalid_argument("missing scheme");

    Url u;
    u.scheme = std::string{s.substr(0, pos_scheme)};
    auto host_port = s.substr(pos_scheme + 3);

    auto pos_colon = host_port.rfind(':');
    if (pos_colon == std::string_view::npos)
        throw std::invalid_argument("missing port");

    u.host = std::string{host_port.substr(0, pos_colon)};
    u.port = static_cast<uint16_t>(std::stoi(
        std::string{host_port.substr(pos_colon + 1)}));
    return u;
}
} // namespace dmw::transport
