/*
 * Dettlaff Blaster Control Module for Betaflight
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h> // Include standard I/O for printf (for SITL debugging)
#include <string.h> // For memcpy
#include <math.h>   // For lrintf
#include <inttypes.h> // For PRIu32 macro

#include "platform.h"

// Include Betaflight HAL and drivers
#include "common/time.h" // For micros() and millis() - types like timeUs_t
#include "drivers/time.h"  // For micros() and millis() - DECLARATIONS
#include "common/maths.h" // For constrain
#include "drivers/io.h" // For IO functions
#include "drivers/io_types.h" // For ioConfig_t enum (IOCFG_IPU)
#include "drivers/motor.h"
#include "drivers/system.h"
#include "drivers/dshot.h" // For DShot constants
#include "drivers/dshot_command.h" // For DShot commands, if needed later
#include "drivers/resource.h" // For resourceOwner_e (OWNER_SYSTEM)
#include "io/beeper.h" // For beeper() and BEEPER_RX_SET
#include "sensors/battery.h"
#include "flight/mixer.h" // For access to the 'motor' array

// Dettlaff specific includes
#include "dettlaff.h"
#include "CONFIGURATION.h" // Dettlaff specific configuration

// --- Static Global Variables ---
static uint32_t loopStartTimer_us = 0; // Updated in dettlaffUpdate
// static uint32_t loopTime_us = 0;       // REMOVED - Unused variable
static uint32_t time_ms = 0;           // Milliseconds since boot

static flywheelState_t flywheelState = STATE_IDLE;
static uint32_t lastRevTime_ms = 0;     // For calculating idling timeout
static uint32_t pusherTimer_ms = 0;     // Timer for pusher solenoid timing
static uint32_t revStartTime_us = 0;    // When rev switch was pressed

// Target RPM/Throttle values (adapt based on protocol in use)
// For simplicity, we'll store the target DShot/PWM values directly
static uint16_t targetFlywheelValue[2] = { 0, 0 }; // Motor 0 and 1
static uint16_t targetPusherValue = PUSHER_SOLENOID_OFF_VALUE; // Motor 2

// Firing mode related (adapt based on MODE_SWITCH_PINIO_INDEX)
static firingMode_e currentFiringMode = DEFAULT_FIRING_MODE;
static int16_t shotsToFire = 0;
static bool firing = false;             // Pusher motor/solenoid is active
static bool pusherDwelling = false;     // Currently in pusher dwell state (closed loop only)
static bool reverseBraking = false;     // Pusher reverse braking active (closed loop only)

// Switch states with debouncing info
static switchState_t triggerSwitch;
static switchState_t revSwitch;

// Battery monitoring
static uint16_t batteryVoltage_10mV = 0; // Voltage in 10mV units (0.01V)

// --- Helper Functions ---

// Debounce a switch reading
static void updateSwitchState(switchState_t *sw, uint32_t currentTimeUs)
{
    // Check if the ioTag is valid before reading
    if (sw->ioTag == IO_TAG_NONE) {
        sw->pressed = false;
        sw->released = false;
        return;
    }

    IO_t io = IOGetByTag(sw->ioTag);
    // Need to check if IOGetByTag actually returned a valid pin, especially for SITL
    if (!io) {
        sw->pressed = false;
        sw->released = false;
        return;
    }
    bool newState = IORead(io); // Read the raw pin state

    sw->pressed = false;
    sw->released = false;

    if (newState != sw->previousState) {
        // State changed, reset timer
        sw->lastChangeTimeUs = currentTimeUs;
    } else if (newState != sw->currentState && (currentTimeUs - sw->lastChangeTimeUs) >= DEBOUNCE_DELAY_US) {
        // State has been stable for the debounce period
        // Assuming Active Low switches (pressed == false)
        if (!newState && sw->currentState) { // Transition HIGH -> LOW (Pressed)
            sw->pressed = true;
        } else if (newState && !sw->currentState) { // Transition LOW -> HIGH (Released)
            sw->released = true;
        }
        sw->currentState = newState;
        DETTLAFF_DEBUG_PRINTF("Switch %x changed to %d (pressed=%d, released=%d)\n", sw->ioTag, sw->currentState, sw->pressed, sw->released);
    }

    sw->previousState = newState;
}

// Initialize a switch state structure using a specific ioTag
static void initSwitch(switchState_t *sw, ioTag_t tag, resourceOwner_e owner, uint8_t ownerIndex)
{
    sw->ioTag = tag;
    if (sw->ioTag == IO_TAG_NONE) {
        sw->currentState = true; // Default to inactive (released) state if no tag
        sw->previousState = true;
        sw->lastChangeTimeUs = micros(); // Use micros()
        sw->pressed = false;
        sw->released = false;
        // Use PRIu32 for uint32_t portability
        printf("Dettlaff: Skipped init for switch tag NONE (Time: %" PRIu32 "us)\n", sw->lastChangeTimeUs);
        return;
    }

    // Configure the GPIO pin
    IO_t io = IOGetByTag(sw->ioTag);
    if (io) {
        IOInit(io, owner, ownerIndex);
        // Configure as Input with Pull-up using IOCFG_IPU
        IOConfigGPIO(io, IOCFG_IPU); // Use IOCFG_IPU from drivers/io_types.h
    } else {
        // Handle error: Pin tag not found or invalid for this target
        printf("ERROR: Dettlaff initSwitch failed for tag %x (Time: %" PRIu32 "us)\n", sw->ioTag, sw->lastChangeTimeUs);
        sw->ioTag = IO_TAG_NONE; // Mark as invalid
        sw->currentState = true;
        sw->previousState = true;
        sw->lastChangeTimeUs = micros(); // Use micros()
        sw->pressed = false;
        sw->released = false;
        return;
    }

    // Initial read after configuration
    // Assuming switches are Active Low (pulled HIGH, go LOW when pressed)
    sw->currentState = IORead(io);
    sw->previousState = sw->currentState;
    sw->lastChangeTimeUs = micros(); // Use micros()
    sw->pressed = false;
    sw->released = false;
     printf("Dettlaff: Initialized switch on tag %x, initial state: %d (Time: %" PRIu32 "us)\n", sw->ioTag, sw->currentState, sw->lastChangeTimeUs);
}

// --- Core Functions ---

void dettlaffInit(void)
{
    // Initialize switch states using specific IO tags
    // Use OWNER_SYSTEM from drivers/resource.h for internal module resources
    initSwitch(&triggerSwitch, TRIGGER_SWITCH_PIN_TAG, OWNER_SYSTEM, 0);
    initSwitch(&revSwitch, REV_SWITCH_PIN_TAG, OWNER_SYSTEM, 1);

    // Initialize state variables
    flywheelState = STATE_IDLE;
    time_ms = millis(); // Use millis()
    lastRevTime_ms = time_ms; // Prevent immediate idle timeout
    pusherTimer_ms = time_ms;
    revStartTime_us = micros(); // Use micros()
    targetFlywheelValue[0] = DSHOT_CMD_MOTOR_STOP; // Ensure motors start stopped
    targetFlywheelValue[1] = DSHOT_CMD_MOTOR_STOP;
    targetPusherValue = PUSHER_SOLENOID_OFF_VALUE;
    shotsToFire = 0;
    firing = false;
    pusherDwelling = false;
    reverseBraking = false;

    // Get initial battery voltage
    batteryVoltage_10mV = getBatteryVoltage();

    currentFiringMode = DEFAULT_FIRING_MODE;

    // Use %lu for time_ms (uint32_t) - NOW USING PRIu32 for portability
    // Use %x for ioTag_t (likely uint16_t or uint32_t, %x is safer)
    printf("Dettlaff module initialized at %" PRIu32 " ms (Switches: Trig=%x, Rev=%x)\n",
           time_ms, triggerSwitch.ioTag, revSwitch.ioTag);
    fflush(stdout); // Ensure output is flushed immediately for SITL

    // Ensure motors are initially off
    motor[0] = targetFlywheelValue[0];
    motor[1] = targetFlywheelValue[1];
    motor[PUSHER_SOLENOID_MOTOR_INDEX] = targetPusherValue;
}

void dettlaffUpdate(uint32_t currentTimeUs)
{
    loopStartTimer_us = currentTimeUs;
    time_ms = millis(); // Update time in milliseconds using millis()

    // 1. Read Inputs and Debounce
    updateSwitchState(&triggerSwitch, currentTimeUs);
    updateSwitchState(&revSwitch, currentTimeUs);

    // 2. Read Battery Voltage
    batteryVoltage_10mV = getBatteryVoltage();

    // 3. Update State Machine
    // Switches are active LOW (false when pressed)
    bool revSwitchIsPressed = revSwitch.currentState == false;
    bool triggerSwitchIsPressed = triggerSwitch.currentState == false;

    // Handle shotsToFire based on trigger and firing mode
    if (triggerSwitch.pressed) { // Use debounced pressed edge
         DETTLAFF_DEBUG_PRINTF("Trigger pressed\n"); // time_ms included by macro
        if (currentFiringMode == FIREMODE_FULL_AUTO) {
            shotsToFire = 1; // Continuous firing while held
        } else if (currentFiringMode == FIREMODE_BURST) {
             shotsToFire += BURST_SHOT_COUNT; // Add burst count
             // Optional: Clamp shotsToFire to max burst count if desired
        } else { // Semi-auto
            shotsToFire += 1; // Add one shot
        }
    } else if (currentFiringMode == FIREMODE_FULL_AUTO && !triggerSwitchIsPressed) {
         // Stop continuous firing if trigger released in full auto
         shotsToFire = 0;
    }

    switch (flywheelState) {
        case STATE_IDLE:
            // Check for low battery cutoff (using Betaflight state)
            if (getBatteryState() == BATTERY_CRITICAL && targetFlywheelValue[0] == DSHOT_CMD_MOTOR_STOP) {
                 DETTLAFF_DEBUG_PRINTF("Battery critical, staying idle.\n"); // time_ms included by macro
            }

            if (revSwitchIsPressed) {
                revStartTime_us = currentTimeUs;
                targetFlywheelValue[0] = FLYWHEEL_SPEED_HIGH; // Set target speed
                targetFlywheelValue[1] = FLYWHEEL_SPEED_HIGH;
                lastRevTime_ms = time_ms;
                flywheelState = STATE_ACCELERATING;
                DETTLAFF_DEBUG_PRINTF("Rev pressed, -> ACCELERATING\n"); // time_ms included by macro
            } else {
                // Stop flywheels if they were running
                if (targetFlywheelValue[0] != DSHOT_CMD_MOTOR_STOP) {
                     DETTLAFF_DEBUG_PRINTF("Stopping flywheels\n"); // time_ms included by macro
                }
                targetFlywheelValue[0] = DSHOT_CMD_MOTOR_STOP;
                targetFlywheelValue[1] = DSHOT_CMD_MOTOR_STOP;
            }
            break;

        case STATE_ACCELERATING:
             // Check if flywheels are spun up (using time delay for now)
             // TODO: Replace with RPM check if using bidirectional DShot/ESC telemetry
             if (time_ms > lastRevTime_ms + FLYWHEEL_RAMP_UP_TIME_MS) {
                 flywheelState = STATE_FULLSPEED;
                 DETTLAFF_DEBUG_PRINTF("Ramp up complete, -> FULLSPEED\n"); // time_ms included by macro
             }

             // If rev switch released during spin-up, go back to idle
             if (!revSwitchIsPressed) {
                 flywheelState = STATE_IDLE;
                 DETTLAFF_DEBUG_PRINTF("Rev released during ramp, -> IDLE\n"); // time_ms included by macro
             }
             // Keep target speed high
             targetFlywheelValue[0] = FLYWHEEL_SPEED_HIGH;
             targetFlywheelValue[1] = FLYWHEEL_SPEED_HIGH;
             lastRevTime_ms = time_ms; // Keep updating while rev held
            break;

        case STATE_FULLSPEED:
            if (!revSwitchIsPressed && shotsToFire <= 0 && !firing) {
                flywheelState = STATE_IDLE;
                DETTLAFF_DEBUG_PRINTF("Rev released, no shots/firing, -> IDLE\n"); // time_ms included by macro
            } else {
                lastRevTime_ms = time_ms; // Keep flywheels revved

                // Pusher Logic (Solenoid Open Loop Example)
                if (shotsToFire > 0 && !firing && (time_ms - pusherTimer_ms) > PUSHER_RETRACTION_TIME_MS) {
                    targetPusherValue = PUSHER_SOLENOID_ON_VALUE;
                    firing = true;
                    if (currentFiringMode != FIREMODE_FULL_AUTO) { // Decrement shots unless full-auto
                       shotsToFire--;
                    }
                    pusherTimer_ms = time_ms;
                    DETTLAFF_DEBUG_PRINTF("Firing shot, extending pusher. shots left: %d\n", shotsToFire); // time_ms included by macro
                    beeper(BEEPER_RX_SET); // Short confirmation beep, defined in io/beeper.h
                } else if (firing && (time_ms - pusherTimer_ms) > PUSHER_EXTENSION_TIME_MS) {
                    targetPusherValue = PUSHER_SOLENOID_OFF_VALUE;
                    firing = false;
                    pusherTimer_ms = time_ms; // Start retraction timer
                    DETTLAFF_DEBUG_PRINTF("Retracting pusher\n"); // time_ms included by macro
                }
            }
             // Keep target speed high
             targetFlywheelValue[0] = FLYWHEEL_SPEED_HIGH;
             targetFlywheelValue[1] = FLYWHEEL_SPEED_HIGH;
            break;
    }

    // 4. Update Motor Outputs
    // Write target values to the motor array for the mixer
    motor[0] = targetFlywheelValue[0];
    motor[1] = targetFlywheelValue[1];
    motor[PUSHER_SOLENOID_MOTOR_INDEX] = targetPusherValue;
}