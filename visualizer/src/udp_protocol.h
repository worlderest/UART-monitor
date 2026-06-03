#ifndef UMON_UDP_PROTOCOL_H
#define UMON_UDP_PROTOCOL_H

#include <stdint.h>

#define UMON_UDP_MAGIC "UMON"
#define UMON_UDP_VERSION 1u

#pragma pack(push, 1)
typedef struct UmonUdpFrame {
    char magic[4];
    uint8_t version;
    uint8_t seq_cnt;
    uint8_t sample_idx;
    uint8_t reserved;
    uint64_t timestamp_ms;
    float upper_arm[4];
    float forearm[4];
    float hand_palm[4];
    uint8_t upper_arm_stale;
    uint8_t forearm_stale;
    uint8_t hand_palm_stale;
    uint8_t reserved2;
    float hand_palm_pitch_deg;
    float hand_palm_pitch_dps;
} UmonUdpFrame;
#pragma pack(pop)

_Static_assert(sizeof(UmonUdpFrame) == 76, "UmonUdpFrame must stay 76 bytes");

#endif
