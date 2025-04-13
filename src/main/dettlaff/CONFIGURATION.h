#pragma once

#include "drivers/io_types.h"    // For ioTag_t
#include "drivers/io_def.h"      // For IO_TAG macro

// --- Pin Configuration ---
// Define specific IO tags for switches.
#ifdef SIMULATOR_BUILD
// SITL doesn't have real GPIOs PB0/PB1
#define TRIGGER_SWITCH_PIN_TAG      IO_TAG_NONE
#define REV_SWITCH_PIN_TAG          IO_TAG_NONE
#define CYCLE_SWITCH_PIN_TAG        IO_TAG_NONE
#define SELECT0_PIN_TAG             IO_TAG_NONE
#define SELECT1_PIN_TAG             IO_TAG_NONE
#define SELECT2_PIN_TAG             IO_TAG_NONE
#else
// *** Verify these pins match your specific board's outputs! ***
#define TRIGGER_SWITCH_PIN_TAG      IO_TAG(PB0) // Typically MOTOR 5 on many F405 boards
#define REV_SWITCH_PIN_TAG          IO_TAG(PB1) // Typically MOTOR 6 on many F405 boards
#define CYCLE_SWITCH_PIN_TAG        IO_TAG_NONE // Pusher motor home switch
#define SELECT0_PIN_TAG             IO_TAG_NONE // optional for select fire
#define SELECT1_PIN_TAG             IO_TAG_NONE // optional for select fire
#define SELECT2_PIN_TAG             IO_TAG_NONE // optional for select fire
#endif

// Outputs (Using standard motor outputs)
// Flywheels use MOTOR 1 and MOTOR 2 (Indices 0 and 1) - Handled by Betaflight motor mixer
#define PUSHER_SOLENOID_MOTOR_INDEX 2 // Corresponds to MOTOR 3 Resource Index

// --- Flywheel Settings ---
// If variableFPS is true, the following settings are set on boot and locked
// Otherwise, it always uses the first mode
typedef bool variableFPS_t;
variableFPS_t variableFPS = true;

// RPM settings for different firing modes (high, medium, low)
// Groups are firing mode 0, 1, 2, and the 4 elements in each group are individual motor RPM
int32_t revRPMset[3][4] = { 
    { 40000, 40000, 40000, 40000 }, 
    { 25000, 25000, 25000, 25000 }, 
    { 14000, 14000, 14000, 14000 } 
};

// How long to idle the flywheels after releasing the trigger (milliseconds)
uint32_t idleTimeSet_ms[3] = { 30000, 5000, 2000 };

// Delay to allow flywheels to spin up before firing dart
uint32_t firingDelaySet_ms[3] = { 150, 125, 100 };

// Delay to allow flywheels to spin up before firing dart when starting from idle state
uint32_t firingDelayIdleSet_ms[3] = { 125, 100, 80 };

// RPM per ms for spindown
uint32_t spindownSpeed = 30;

// Motor parameters for closed loop control
int32_t motorKv = 3200; // critical for closed loop
int32_t idleRPM[4] = { 500, 500, 500, 500 }; // rpm for flywheel idling

// --- Closed Loop Settings ---
typedef enum {
    OPEN_LOOP_CONTROL,
    TWO_LEVEL_CONTROL,
    PID_CONTROL
} flywheelControlType_t;

flywheelControlType_t flywheelControl = OPEN_LOOP_CONTROL;

// Which motors are hooked up
const bool motorsPresent[4] = {true, true, true, true};

// While idling, fire pusher after firingDelay_ms even before flywheels are up to speed
bool timeOverrideWhenIdling = true;

// If rpm is more than this amount below target, send full throttle
int32_t fullThrottleRpmTolerance = 5000;

// Fire pusher when all flywheels are within this amount of target rpm
int32_t firingRPMTolerance = 10000;

// Overrides firingRPMTolerance for low rpm settings
int32_t minFiringRPM = 10000;

// When not idling, don't fire before this amount of time, even if wheels are up to speed
int32_t minFiringDelaySet_ms[3] = {0, 0, 0};

// Same but when idling
int32_t minFiringDelayIdleSet_ms[3] = {0, 0, 0};

// --- Select Fire Settings ---
// Burst lengths for each firing mode
uint32_t burstLengthSet[3] = { 100, 5, 1 };

typedef enum {
    AUTO,   // stops firing when trigger is released
    BURST,  // always completes the burst
    BINARY  // fires one burst when you pull the trigger and another when you release the trigger
} burstFireType_t;

burstFireType_t burstModeSet[3] = { AUTO, AUTO, BURST };

// for full auto, set burstLength high (50+) and burstMode to AUTO
// for semi auto, set burstLength to 1 and burstMode to BURST
// for burst fire, set burstLength and burstMode to BURST

// if you hold the trigger for more than this amount of time, releasing the trigger will not fire a burst
uint32_t binaryTriggerTimeout_ms = 2000;

typedef enum {
    NO_SELECT_FIRE,
    SWITCH_SELECT_FIRE,
    BUTTON_SELECT_FIRE
} selectFireType_t;

selectFireType_t selectFireType = SWITCH_SELECT_FIRE;

// only for SWITCH_SELECT_FIRE, what mode to select if no pins are connected
uint8_t defaultFiringMode = 1;

// --- Dettlaff Settings ---
// output telemetry over USB serial port for tuning
bool printTelemetry = false;

// Low voltage cutoff (in mV), default is 2.5V per cell * 4 cells
uint32_t lowVoltageCutoff_mv = 2500 * 4;

// --- Pusher Settings ---
typedef enum {
    NO_PUSHER,
    PUSHER_MOTOR_CLOSEDLOOP,
    PUSHER_SOLENOID_OPENLOOP
} pusherType_t;

pusherType_t pusherType = PUSHER_SOLENOID_OPENLOOP;

// if battery voltage is above this, use PWM to reduce pusher voltage
uint32_t pusherVoltage_mv = 13000;

// make motor spin backwards?
bool pusherReverseDirection = false;

// --- Solenoid Settings ---
uint16_t solenoidExtendTime_ms = 20;
uint16_t solenoidRetractTime_ms = 35;

// --- Pusher Motor Settings ---
uint32_t pusherReverseBrakingVoltage_mv = 16000;
uint8_t pusherReversePolarityDuration_ms = 5;

// dwell for this long at the end of each pusher cycle in full auto/burst
uint32_t pusherDwellTime_ms = 0;

// if true then the pusher brakes during its dwell time, if false it coasts
bool pusherBrakeOnDwell = false;

// --- Advanced Settings ---
// For PUSHER_MOTOR_CLOSEDLOOP, time to run motor without cycle switch update before deciding motor is stalled
uint16_t pusherStallTime_ms = 750;

// Invert switch signals?
bool revSwitchNormallyClosed = false;
bool triggerSwitchNormallyClosed = false;
bool cycleSwitchNormallyClosed = false;

// Debounce time for switches (milliseconds)
uint16_t debounceTime_ms = 100;
uint16_t pusherDebounceTime_ms = 25;

// --- PID Settings ---
float KP = 1.5;
float KI = 0.0;
float KD = 0.5;

// --- Misc ---
// #define DEBUG_DETTLAFF // Uncomment for debug prints