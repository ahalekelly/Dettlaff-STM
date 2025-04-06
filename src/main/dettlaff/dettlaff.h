/*
 * Dettlaff Blaster Control Module
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "drivers/io_types.h" // For ioTag_t

// Dettlaff - Function prototypes
void dettlaffInit(void);
void dettlaffUpdate(uint32_t currentTimeUs);

// State definitions mirroring ESP32 code
typedef enum {
    STATE_IDLE,
    STATE_ACCELERATING,
    STATE_FULLSPEED
} flywheelState_t;

// Structure for switch debouncing
typedef struct {
    ioTag_t ioTag;
    bool currentState;
    bool previousState;
    uint32_t lastChangeTimeUs;
    bool pressed; // True if a transition from released to pressed occurred
    bool released; // True if a transition from pressed to released occurred
} switchState_t;

#ifdef DEBUG_DETTLAFF
#define DETTLAFF_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DETTLAFF_DEBUG_PRINTF(...) ((void)0)
#endif