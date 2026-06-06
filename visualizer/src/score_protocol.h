#ifndef UMON_SCORE_PROTOCOL_H
#define UMON_SCORE_PROTOCOL_H

#include <stdint.h>

#define UMON_SCORE_MAGIC "USCR"
#define UMON_SCORE_VERSION 1u

#pragma pack(push, 1)
typedef struct UmonScoreEvent {
    char magic[4];
    uint8_t version;
    uint8_t reserved[3];
    uint64_t shot_timestamp_ms;
    float total_score;
    float stability_jitter_deg;
    float max_muzzle_jump_deg;
    float recovery_time_ms;
} UmonScoreEvent;
#pragma pack(pop)

_Static_assert(sizeof(UmonScoreEvent) == 32, "UmonScoreEvent must stay 32 bytes");

#endif
