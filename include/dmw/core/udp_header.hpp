#pragma once
#include <cstdint>

namespace dmw::core {

#pragma pack(push,1)
struct UdpHdr        // 16 octets
{
    uint32_t seq;      // identifiant snapshot
    uint32_t tot;      // taille totale blob
    uint32_t off;      // offset fragment
    uint16_t len;      // longueur fragment
    uint8_t  flags;    // 0x01=snapshot 0x02=delta 0x04=gzip
    uint8_t  drone_id; // 0-255
};
#pragma pack(pop)

inline constexpr uint8_t FLAG_SNAPSHOT = 0x01;
inline constexpr uint8_t FLAG_DELTA    = 0x02;
inline constexpr uint8_t FLAG_GZIP     = 0x04;
inline constexpr uint16_t DEFAULT_MTU  = 1300;

} // namespace dmw::core
