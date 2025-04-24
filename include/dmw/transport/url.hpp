#pragma once
#include <cstdint>
#include <string>
#include <string_view>

namespace dmw::transport
{
struct Url
{
    std::string scheme;   // "udp", "quic"…
    std::string host;     // IP ou FQDN
    uint16_t    port{0};
};

Url parse(std::string_view s);   ///< lève std::invalid_argument si invalide
} // namespace dmw::transport
