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

// --- State Definitions ---

// Flywheel state
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
    bool isPressedState; // True if the button is considered pressed when low (normal), false if inverted
} switchState_t;

// Driver types for the pusher
typedef enum {
    DRIVER_COAST,
    DRIVER_BRAKE,
    DRIVER_FORWARD,
    DRIVER_REVERSE
} driverCommand_t;

// Structure for a generic motor driver
typedef struct {
    void (*drive)(float percent, bool reverse);
    void (*coast)(void);
    void (*brake)(void);
} motorDriver_t;

// For DShot telemetry and motor control
typedef struct {
    int32_t rpm;           // Current RPM from telemetry
    int32_t targetRPM;     // Target RPM
    int32_t firingRPM;     // RPM threshold for firing
    int32_t fullThrottleRpmThreshold; // Threshold for full throttle
    int32_t throttleValue; // Current throttle value (0-1999)
    bool active;           // Whether this motor is active
} motorState_t;

// PID control variables
typedef struct {
    int32_t error;
    int32_t errorPrior;
    int32_t integral;
    int32_t output;
} pidState_t;

#ifdef DEBUG_DETTLAFF
#define DETTLAFF_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DETTLAFF_DEBUG_PRINTF(...) ((void)0)
#endif

// Define maximum value for debug printing of telemetry
#define DETTLAFF_MAX_DEBUG_RPM 50000