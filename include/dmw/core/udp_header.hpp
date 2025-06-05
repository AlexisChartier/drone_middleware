#pragma once
#include <cstdint>

namespace dmw::core {
#pragma pack(push,1)
struct UdpHeader {
    uint32_t seq;      // identifiant du scan complet
    uint32_t tot;      // taille totale du blob octomap
    uint32_t off;      // offset du fragment
    uint16_t len;      // longueur de ce fragment
    uint8_t  flags;    // bit0 = snapshot, bit1 = delta
    uint8_t  drone_id; // 0‑255
};
#pragma pack(pop)

enum UdpFlags : uint8_t {
    SNAPSHOT = 1,
    DELTA    = 2
};
} // namespace dmw::core