#pragma once
#include <functional>
#include <map>
#include "itransport.hpp"
#include "url.hpp"

namespace dmw::transport
{
using Creator = std::function<Ptr()>;

class Factory
{
public:
    static Factory& instance();
    void   register_backend(std::string_view scheme, Creator);
    [[nodiscard]] Ptr make(const Url&);

private:
    std::map<std::string, Creator, std::less<>> map_;
};
} // namespace dmw::transport
