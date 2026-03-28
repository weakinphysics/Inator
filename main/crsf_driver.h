#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <cstdint>
#include <cstring>
#include <cstdio>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_MAX_FRAME_SIZE        64
#define CRSF_ADDRESS_FLIGHT_CTRL   0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

typedef struct {
    uint16_t ch[16];      // decoded to approximately 1000..2000 us
    bool     failsafe;
    uint32_t last_frame_us;
} crsf_channels_t;

void crsf_init(void);
bool crsf_poll(crsf_channels_t *out);

#ifdef __cplusplus
}
#endif