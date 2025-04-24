#include "dmw/transport/factory.hpp"

namespace dmw::transport
{
Factory& Factory::instance()
{
    static Factory inst;
    return inst;
}

void Factory::register_backend(std::string_view scheme, Creator c)
{
    map_.emplace(scheme, std::move(c));
}

Ptr Factory::make(const Url& u)
{
    auto it = map_.find(u.scheme);
    if (it == map_.end()) return nullptr;
    return (it->second)();
}
} // namespace dmw::transport
