#pragma once
#include <cstdint>
#include <memory>
#include <string_view>

namespace dmw::transport
{
struct PacketView { const void* data; std::size_t size; };

class ITransport
{
public:
    virtual ~ITransport() = default;
    virtual bool connect(std::string_view host, uint16_t port) = 0;
    virtual bool send(PacketView p) noexcept = 0;
    virtual void close() noexcept = 0;
};
using Ptr = std::unique_ptr<ITransport>;
} // namespace dmw::transport
