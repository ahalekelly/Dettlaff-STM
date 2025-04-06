#pragma once

#include "drivers/motor_types.h" // For motorProtocol_e
#include "drivers/io_types.h"    // For ioTag_t
#include "drivers/io_def.h"      // For IO_TAG macro

// --- Pin Configuration ---
// Define specific IO tags for switches.
#ifdef SIMULATOR_BUILD
// SITL doesn't have real GPIOs PB0/PB1
#define TRIGGER_SWITCH_PIN_TAG      IO_TAG_NONE
#define REV_SWITCH_PIN_TAG          IO_TAG_NONE
#else
// *** Verify these pins (PB1, PB0) match your specific board's outputs! ***
#define TRIGGER_SWITCH_PIN_TAG      IO_TAG(PB0) // Typically MOTOR 5 on many F405 boards
#define REV_SWITCH_PIN_TAG          IO_TAG(PB1) // Typically MOTOR 6 on many F405 boards
#endif
#define MODE_SWITCH_PIN_TAG         IO_TAG_NONE // Mode switch disabled as requested

// Outputs (Using standard motor outputs)
// Flywheels use MOTOR 1 and MOTOR 2 (Indices 0 and 1) - Handled by Betaflight motor mixer
#define PUSHER_SOLENOID_MOTOR_INDEX 2 // Corresponds to MOTOR 3 Resource Index

// --- Motor Configuration ---
// Note: Motor protocol (DShot/PWM etc.) should be set via Betaflight Configurator or CLI 'motor_pwm_protocol'.
// The #defines below are mainly for value ranges based on the chosen protocol.
// DShot constants are defined in drivers/dshot.h, avoid redefining them here.

// Flywheel Speeds (Example using DShot range, adjust as needed based on actual motor_pwm_protocol)
// Consider checking motorConfig()->dev.motorProtocol in the code to adapt values.
// Use constants from drivers/dshot.h
#include "drivers/dshot.h"
#define FLYWHEEL_SPEED_IDLE         (DSHOT_MIN_THROTTLE + (uint16_t)(DSHOT_RANGE * 0.10f)) // 10% speed
#define FLYWHEEL_SPEED_LOW          (DSHOT_MIN_THROTTLE + (uint16_t)(DSHOT_RANGE * 0.50f)) // 50% speed
#define FLYWHEEL_SPEED_MEDIUM       (DSHOT_MIN_THROTTLE + (uint16_t)(DSHOT_RANGE * 0.75f)) // 75% speed
#define FLYWHEEL_SPEED_HIGH         (DSHOT_MAX_THROTTLE)                                   // 100% speed

// Flywheel ramp-up time (milliseconds)
#define FLYWHEEL_RAMP_UP_TIME_MS    500

// --- Pusher Configuration ---
// Map pusher ON/OFF state to motor values (Example uses DShot range)
#define PUSHER_SOLENOID_OFF_VALUE   DSHOT_MIN_THROTTLE // Use min throttle for OFF state
#define PUSHER_SOLENOID_ON_VALUE    DSHOT_MAX_THROTTLE // Use max throttle for ON state

// Pusher timing (milliseconds)
#define PUSHER_EXTENSION_TIME_MS    50  // Time solenoid stays extended
#define PUSHER_RETRACTION_TIME_MS   100 // Time needed for pusher to retract before next shot

// --- Firing Modes ---
// Mode switch is disabled, only default mode is used.
typedef enum {
    FIREMODE_SEMI_AUTO,
    FIREMODE_BURST,
    FIREMODE_FULL_AUTO,
    FIREMODE_COUNT
} firingMode_e;

#define DEFAULT_FIRING_MODE FIREMODE_SEMI_AUTO
#define BURST_SHOT_COUNT    3

// --- Debounce ---
#define DEBOUNCE_DELAY_US 50000 // 50ms debounce time for switches

// --- Battery Monitoring ---
// Betaflight handles ADC config. These are thresholds for warnings/cutoffs.
// Values are in 0.01V units (e.g., 330 = 3.30V)
// Use Betaflight's battery settings (batteryConfig_t) via Configurator/CLI.
// These defines are placeholders and might not be directly used if relying on Betaflight state.
#define MIN_CELL_VOLTAGE 330 // 3.3V per cell
#define WARN_CELL_VOLTAGE 350 // 3.5V per cell

// --- Misc ---
// #define DEBUG_DETTLAFF // Uncomment for debug prints