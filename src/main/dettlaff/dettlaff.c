/*
 * Dettlaff Blaster Control Module for Betaflight
 * Ported from ESP32-Dettlaff
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h> // Include standard I/O for printf (for SITL debugging)
#include <string.h> // For memcpy
#include <math.h>   // For lrintf, max, min
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
#include "drivers/resource.h" // For resourceOwner_e (OWNER_SYSTEM)
#include "io/beeper.h" // For beeper() and BEEPER_RX_SET
#include "sensors/battery.h"
#include "flight/mixer.h" // For access to the 'motor' array

// Dettlaff specific includes
#include "dettlaff.h"
#include "CONFIGURATION.h" // Dettlaff specific configuration

// --- State Variables and Constants ---
static uint32_t loopStartTimer_us = 0;  // When the current loop started
static uint32_t loopTime_us = 1000;     // Target loop time in microseconds
static uint32_t time_ms = 0;            // Milliseconds since boot
static uint32_t lastRevTime_ms = 0;     // For calculating idling
static uint32_t pusherTimer_ms = 0;     // Used for various pusher timing
static uint32_t revStartTime_us = 0;    // When rev trigger was activated
static uint32_t triggerTime_ms = 0;     // When trigger was activated for binary mode

// Motor control variables
static int32_t revRPM[4];               // RPM settings for current firing mode
static int32_t idleTime_ms;             // Idle time for current firing mode
static int32_t targetRPM[4] = { 0, 0, 0, 0 }; // Current target RPM for each motor
static int32_t firingRPM[4];            // RPM threshold for firing
static float throttleValue[4] = { 0.0f, 0.0f, 0.0f, 0.0f }; // Scale is 0.0 - 1.0 for Betaflight motors
static uint32_t currentSpindownSpeed = 0; // Current spindown rate

// Firing mode variables
static uint16_t burstLength;            // Number of shots in a burst
static burstFireType_t burstMode;       // Type of burst firing
static int8_t firingMode = 0;           // Current firing mode
static int8_t fpsMode = 0;              // Copy of firingMode locked at boot
static bool fromIdle = false;           // Whether coming from idle state

// Firing state
static int16_t shotsToFire = 0;         // Number of shots left to fire
static flywheelState_t flywheelState = STATE_IDLE;
static bool firing = false;             // Pusher is active
static bool reverseBraking = false;     // Pusher is reverse braking
static bool pusherDwelling = false;     // Pusher is in dwell period

// Battery monitoring
static uint32_t batteryVoltage_mv = 0;  // Battery voltage in millivolts

// Motor status
static int32_t motorRPM[4] = { 0, 0, 0, 0 };  // Current RPM from telemetry
static int32_t fullThrottleRpmThreshold[4] = { 0, 0, 0, 0 }; // RPM threshold for full throttle

// PID control variables
static int32_t PIDError[4];
static int32_t PIDErrorPrior[4];
static int32_t closedLoopRPM[4];
static int32_t PIDOutput[4];
static int32_t PIDIntegral = 0;

// Switch states with debouncing
static switchState_t triggerSwitch;
static switchState_t revSwitch;
static switchState_t cycleSwitch;
static switchState_t select0Switch;
static switchState_t select1Switch;
static switchState_t select2Switch;

// Motor driver implementation for STM32
static motorDriver_t pusherDriver;

// --- Helper Functions ---

// Simple pusher driver implementation using motor outputs
void pusher_drive(float percent, bool reverse) {
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    
    // Map percentage to 0.0-1.0 range for Betaflight motor outputs
    float value = percent / 100.0f;
    
    // For reverse direction, we can use negative values
    // Note: For this to work properly with a brushed motor ESC,
    // you would need to enable the FEATURE_3D in Betaflight
    if (reverse) {
        // Use negative values for reverse
        value = -value;
    }
    
    // Apply value to the motor output
    motor[PUSHER_SOLENOID_MOTOR_INDEX] = value;
    
    DETTLAFF_DEBUG_PRINTF("Pusher driving at %.1f%% %s (motor value: %.3f)\n", 
                          percent, reverse ? "reverse" : "forward", value);
}

void pusher_coast(void) {
    motor[PUSHER_SOLENOID_MOTOR_INDEX] = 0.0f;
    DETTLAFF_DEBUG_PRINTF("Pusher coasting\n");
}

void pusher_brake(void) {
    // For braking, we use motor stop (0.0)
    motor[PUSHER_SOLENOID_MOTOR_INDEX] = 0.0f;
    DETTLAFF_DEBUG_PRINTF("Pusher braking\n");
}

// Debounce a switch reading with configurable pressed state
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
    
    bool rawState = IORead(io); // Read the raw pin state
    bool newState = sw->isPressedState ? !rawState : rawState; // Invert if normally closed

    sw->pressed = false;
    sw->released = false;

    if (newState != sw->previousState) {
        // State changed, reset timer
        sw->lastChangeTimeUs = currentTimeUs;
    } else if (newState != sw->currentState && 
              (currentTimeUs - sw->lastChangeTimeUs) >= (debounceTime_ms * 1000)) {
        // State has been stable for the debounce period
        if (newState && !sw->currentState) { // Transition to pressed state
            sw->pressed = true;
        } else if (!newState && sw->currentState) { // Transition to released state
            sw->released = true;
        }
        sw->currentState = newState;
        DETTLAFF_DEBUG_PRINTF("Switch %x changed to %d (pressed=%d, released=%d)\n", 
                             sw->ioTag, sw->currentState, sw->pressed, sw->released);
    }

    sw->previousState = newState;
}

// Update switch state for cycle switch with its own debounce time
static void updateCycleSwitchState(switchState_t *sw, uint32_t currentTimeUs)
{
    // Similar to updateSwitchState but with cycle-switch specific debounce time
    if (sw->ioTag == IO_TAG_NONE) {
        sw->pressed = false;
        sw->released = false;
        return;
    }

    IO_t io = IOGetByTag(sw->ioTag);
    if (!io) {
        sw->pressed = false;
        sw->released = false;
        return;
    }
    
    bool rawState = IORead(io);
    bool newState = sw->isPressedState ? !rawState : rawState;

    sw->pressed = false;
    sw->released = false;

    if (newState != sw->previousState) {
        sw->lastChangeTimeUs = currentTimeUs;
    } else if (newState != sw->currentState && 
              (currentTimeUs - sw->lastChangeTimeUs) >= (pusherDebounceTime_ms * 1000)) {
        if (newState && !sw->currentState) {
            sw->pressed = true;
        } else if (!newState && sw->currentState) {
            sw->released = true;
        }
        sw->currentState = newState;
        DETTLAFF_DEBUG_PRINTF("Cycle switch %x changed to %d (pressed=%d, released=%d)\n", 
                             sw->ioTag, sw->currentState, sw->pressed, sw->released);
    }

    sw->previousState = newState;
}

// Initialize a switch state structure using a specific ioTag
static void initSwitch(switchState_t *sw, ioTag_t tag, bool inversePressedState, resourceOwner_e owner, uint8_t ownerIndex)
{
    sw->ioTag = tag;
    sw->isPressedState = !inversePressedState; // If inversePressedState, then pressed means HIGH
    
    if (sw->ioTag == IO_TAG_NONE) {
        sw->currentState = false; // Default to inactive (released) state if no tag
        sw->previousState = false;
        sw->lastChangeTimeUs = micros();
        sw->pressed = false;
        sw->released = false;
        printf("Dettlaff: Skipped init for switch tag NONE\n");
        return;
    }

    // Configure the GPIO pin
    IO_t io = IOGetByTag(sw->ioTag);
    if (io) {
        IOInit(io, owner, ownerIndex);
        // Configure as Input with Pull-up using IOCFG_IPU
        IOConfigGPIO(io, IOCFG_IPU);
    } else {
        printf("ERROR: Dettlaff initSwitch failed for tag %x\n", sw->ioTag);
        sw->ioTag = IO_TAG_NONE; // Mark as invalid
        sw->currentState = false;
        sw->previousState = false;
        sw->lastChangeTimeUs = micros();
        sw->pressed = false;
        sw->released = false;
        return;
    }

    // Initial read after configuration
    bool rawState = IORead(io);
    sw->currentState = sw->isPressedState ? !rawState : rawState; // Apply polarity
    sw->previousState = sw->currentState;
    sw->lastChangeTimeUs = micros();
    sw->pressed = false;
    sw->released = false;
    printf("Dettlaff: Initialized switch on tag %x, initial state: %d (inverted: %d)\n", 
           sw->ioTag, sw->currentState, inversePressedState);
}

// Update the current firing mode based on select fire switches
static void updateFiringMode(void) 
{
    if (selectFireType == NO_SELECT_FIRE) {
        return;
    }
    
    // Update switch states
    updateSwitchState(&select0Switch, micros());
    updateSwitchState(&select1Switch, micros());
    updateSwitchState(&select2Switch, micros());
    
    // Check each switch
    if (select0Switch.ioTag != IO_TAG_NONE && select0Switch.currentState) {
        firingMode = 0;
        return;
    }
    
    if (select1Switch.ioTag != IO_TAG_NONE && select1Switch.currentState) {
        firingMode = 1;
        return;
    }
    
    if (select2Switch.ioTag != IO_TAG_NONE && select2Switch.currentState) {
        firingMode = 2;
        return;
    }
    
    // Default mode if no switches are pressed
    if (selectFireType == SWITCH_SELECT_FIRE) {
        firingMode = defaultFiringMode;
    }
}

// Convert battery voltage from Betaflight's 0.01V units to mV
static uint32_t batteryVoltageToMillivolts(uint16_t voltage_10mV) {
    return (uint32_t)voltage_10mV * 10;
}

// --- Core Functions ---

void dettlaffInit(void)
{
    // Initialize the pusher driver
    pusherDriver.drive = pusher_drive;
    pusherDriver.coast = pusher_coast;
    pusherDriver.brake = pusher_brake;
    
    // Initialize switch states
    initSwitch(&triggerSwitch, TRIGGER_SWITCH_PIN_TAG, triggerSwitchNormallyClosed, OWNER_SYSTEM, 0);
    initSwitch(&revSwitch, REV_SWITCH_PIN_TAG, revSwitchNormallyClosed, OWNER_SYSTEM, 1);
    initSwitch(&cycleSwitch, CYCLE_SWITCH_PIN_TAG, cycleSwitchNormallyClosed, OWNER_SYSTEM, 2);
    
    // Initialize select fire switches if enabled
    if (selectFireType != NO_SELECT_FIRE) {
        initSwitch(&select0Switch, SELECT0_PIN_TAG, false, OWNER_SYSTEM, 3);
        initSwitch(&select1Switch, SELECT1_PIN_TAG, false, OWNER_SYSTEM, 4);
        initSwitch(&select2Switch, SELECT2_PIN_TAG, false, OWNER_SYSTEM, 5);
    }

    // Initialize state variables
    flywheelState = STATE_IDLE;
    time_ms = millis();
    lastRevTime_ms = 0;
    pusherTimer_ms = time_ms;
    revStartTime_us = micros();
    
    // Initialize firing mode
    if (variableFPS) {
        updateFiringMode();
    }
    
    fpsMode = firingMode;
    printf("Dettlaff module initialized, fpsMode: %d\n", fpsMode);
    
    // Initialize motor RPM settings
    for (int i = 0; i < 4; i++) {
        if (motorsPresent[i]) {
            revRPM[i] = revRPMset[fpsMode][i];
            firingRPM[i] = MAX(revRPM[i] - firingRPMTolerance, minFiringRPM);
            fullThrottleRpmThreshold[i] = revRPM[i] - fullThrottleRpmTolerance;
        }
    }
    idleTime_ms = idleTimeSet_ms[fpsMode];
    
    // Get initial battery voltage
    batteryVoltage_mv = batteryVoltageToMillivolts(getBatteryVoltage());
    
    // Initialize flywheel motors to off
    for (int i = 0; i < 4; i++) {
        if (motorsPresent[i]) {
            motor[i] = 0.0f; // Motors off (0.0f)
        }
    }
    
    pusherDriver.coast(); // Ensure pusher is in coast state initially
    
    printf("Dettlaff module initialized (Battery: %lu mV)\n", (unsigned long)batteryVoltage_mv);
    fflush(stdout); // Ensure output is flushed for SITL
}

void dettlaffUpdate(uint32_t currentTimeUs)
{
    loopStartTimer_us = currentTimeUs;
    time_ms = millis();
    
    // 1. Read Inputs and Debounce
    updateSwitchState(&revSwitch, currentTimeUs);
    updateSwitchState(&triggerSwitch, currentTimeUs);
    updateCycleSwitchState(&cycleSwitch, currentTimeUs);
    
    // 2. Update firing mode and burst settings
    updateFiringMode();
    burstLength = burstLengthSet[firingMode];
    burstMode = burstModeSet[firingMode];
    
    // 3. Handle trigger logic
    // Binary mode: pulled AND released within timeout
    if (triggerSwitch.pressed || 
        (burstMode == BINARY && triggerSwitch.released && time_ms < triggerTime_ms + binaryTriggerTimeout_ms)) {
        
        DETTLAFF_DEBUG_PRINTF("Trigger %s, burstMode %d\n", 
                              triggerSwitch.pressed ? "pressed" : "released (binary)", burstMode);
        
        triggerTime_ms = time_ms;
        
        if (burstMode == AUTO) {
            shotsToFire = burstLength;
        } else {
            if (shotsToFire < burstLength || shotsToFire == 1) {
                shotsToFire += burstLength;
            }
        }
        DETTLAFF_DEBUG_PRINTF("shotsToFire now %d\n", shotsToFire);
    } else if (triggerSwitch.released) {
        if (burstMode == AUTO && shotsToFire > 1) {
            shotsToFire = 1; // Stop after completing current shot in AUTO mode
        }
    }
    
    // 4. Update battery voltage
    batteryVoltage_mv = batteryVoltageToMillivolts(getBatteryVoltage());
    
    // 5. State machine update
    switch (flywheelState) {
        case STATE_IDLE:
            // Check for low battery cutoff
            if (batteryVoltage_mv < lowVoltageCutoff_mv && throttleValue[0] == 0.0f && loopStartTimer_us > 2000000) {
                DETTLAFF_DEBUG_PRINTF("Battery low, shutting down! %lu mV\n", (unsigned long)batteryVoltage_mv);
                // We can't deep sleep like ESP32, but we can put the motors in a safe state
                for (int i = 0; i < 4; i++) {
                    if (motorsPresent[i]) {
                        motor[i] = 0.0f; // Motor off
                    }
                }
                pusherDriver.coast();
                return;
            }
            
            // Check if we need to start spinning up
            if (shotsToFire > 0 || revSwitch.currentState) {
                revStartTime_us = loopStartTimer_us;
                memcpy(targetRPM, revRPM, sizeof(targetRPM)); // Copy revRPM to targetRPM
                lastRevTime_ms = time_ms;
                flywheelState = STATE_ACCELERATING;
                currentSpindownSpeed = 0;
                DETTLAFF_DEBUG_PRINTF("Flywheels accelerating\n");
            } else if (time_ms < lastRevTime_ms + idleTime_ms && lastRevTime_ms > 0) {
                // Gradually spin down to idle
                if (currentSpindownSpeed < spindownSpeed) {
                    currentSpindownSpeed += 1;
                }
                
                for (int i = 0; i < 4; i++) {
                    if (motorsPresent[i]) {
                        targetRPM[i] = MAX(targetRPM[i] - ((int32_t)((currentSpindownSpeed * loopTime_us) / 1000)), 
                                           idleRPM[i]);
                    }
                }
            } else {
                // Complete shutdown
                if (currentSpindownSpeed < spindownSpeed) {
                    currentSpindownSpeed += 1;
                }
                
                for (int i = 0; i < 4; i++) {
                    if (motorsPresent[i] && targetRPM[i] != 0) {
                        targetRPM[i] = MAX(targetRPM[i] - ((int32_t)((currentSpindownSpeed * loopTime_us) / 1000)), 0);
                    }
                }
                PIDIntegral = 0;
                fromIdle = false;
            }
            break;
            
        case STATE_ACCELERATING:
            // Check if closed-loop control is ready to fire
            if (flywheelControl != OPEN_LOOP_CONTROL && 
                time_ms > lastRevTime_ms + (fromIdle ? minFiringDelayIdleSet_ms[fpsMode] : minFiringDelaySet_ms[fpsMode])) {
                
                // Check if all active motors have reached firing RPM
                if ((!motorsPresent[0] || motorRPM[0] > firingRPM[0]) &&
                    (!motorsPresent[1] || motorRPM[1] > firingRPM[1]) &&
                    (!motorsPresent[2] || motorRPM[2] > firingRPM[2]) &&
                    (!motorsPresent[3] || motorRPM[3] > firingRPM[3])) {
                    
                    flywheelState = STATE_FULLSPEED;
                    fromIdle = true;
                    DETTLAFF_DEBUG_PRINTF("Flywheels at full speed (RPM check)\n");
                } else if (loopStartTimer_us - revStartTime_us > 2000000) {
                    // Timeout if motors don't reach speed in 2 seconds
                    flywheelState = STATE_IDLE;
                    shotsToFire = 0;
                    DETTLAFF_DEBUG_PRINTF("Error! Flywheels failed to reach target speed!\n");
                }
            }
            
            // Check if open-loop control or override is ready to fire
            if ((flywheelControl == OPEN_LOOP_CONTROL || 
                 (timeOverrideWhenIdling && fromIdle &&
                  (!motorsPresent[0] || motorRPM[0] > 100) &&
                  (!motorsPresent[1] || motorRPM[1] > 100) &&
                  (!motorsPresent[2] || motorRPM[2] > 100) &&
                  (!motorsPresent[3] || motorRPM[3] > 100))) && 
                time_ms > lastRevTime_ms + (fromIdle ? firingDelayIdleSet_ms[fpsMode] : firingDelaySet_ms[fpsMode])) {
                
                flywheelState = STATE_FULLSPEED;
                fromIdle = true;
                DETTLAFF_DEBUG_PRINTF("Flywheels at full speed (time delay)\n");
            }
            break;
            
        case STATE_FULLSPEED:
            // Check if we should return to idle
            if (!revSwitch.currentState && shotsToFire == 0 && !firing) {
                flywheelState = STATE_IDLE;
                DETTLAFF_DEBUG_PRINTF("Returning to idle\n");
            } else if (shotsToFire > 0 || firing) {
                // Keep flywheels active
                lastRevTime_ms = time_ms;
                
                // Handle the pusher based on pusher type
                switch (pusherType) {
                    case PUSHER_MOTOR_CLOSEDLOOP:
                        if (shotsToFire > 0 && !firing) {
                            // Start pusher stroke
                            pusherDriver.drive(100.0f * pusherVoltage_mv / batteryVoltage_mv, pusherReverseDirection);
                            firing = true;
                            pusherTimer_ms = time_ms;
                            DETTLAFF_DEBUG_PRINTF("Pusher stroke starting\n");
                        } else if (firing && cycleSwitch.pressed) {
                            // Pusher reached rear position
                            shotsToFire = MAX(0, shotsToFire - 1);
                            pusherTimer_ms = time_ms;
                            DETTLAFF_DEBUG_PRINTF("Pusher reached rear, shots left: %d\n", shotsToFire);
                            
                            if (shotsToFire <= 0) {
                                // End of firing sequence, apply braking
                                if (pusherReversePolarityDuration_ms > 0) {
                                    pusherDriver.drive(100.0f * pusherReverseBrakingVoltage_mv / batteryVoltage_mv, 
                                                      !pusherReverseDirection);
                                    reverseBraking = true;
                                    DETTLAFF_DEBUG_PRINTF("Reverse braking started\n");
                                } else {
                                    pusherDriver.brake();
                                    firing = false;
                                    flywheelState = STATE_IDLE;
                                    DETTLAFF_DEBUG_PRINTF("Firing complete, braking\n");
                                }
                            } else if (pusherDwellTime_ms > 0) {
                                // Dwell period to slow down rate of fire
                                if (pusherBrakeOnDwell) {
                                    pusherDriver.brake();
                                } else {
                                    pusherDriver.coast();
                                }
                                pusherDwelling = true;
                                DETTLAFF_DEBUG_PRINTF("Pusher dwelling\n");
                            }
                        } else if (pusherDwelling) {
                            // Check if dwell time is complete
                            if (time_ms - pusherTimer_ms > pusherDwellTime_ms) {
                                pusherDriver.drive(100.0f * pusherVoltage_mv / batteryVoltage_mv, pusherReverseDirection);
                                pusherDwelling = false;
                                DETTLAFF_DEBUG_PRINTF("Dwell complete, resuming push\n");
                            }
                        } else if (reverseBraking) {
                            // Handle reverse braking
                            if (shotsToFire >= 1) {
                                // Resume firing if shots were added
                                pusherDriver.drive(100.0f * pusherVoltage_mv / batteryVoltage_mv, pusherReverseDirection);
                                firing = true;
                                pusherTimer_ms = time_ms;
                                reverseBraking = false;
                                DETTLAFF_DEBUG_PRINTF("Ending reverse braking, resuming firing\n");
                            } else if (time_ms > pusherTimer_ms + pusherReversePolarityDuration_ms) {
                                // End reverse braking after duration
                                pusherDriver.brake();
                                reverseBraking = false;
                                firing = false;
                                flywheelState = STATE_IDLE;
                                DETTLAFF_DEBUG_PRINTF("Reverse braking complete\n");
                            }
                        } else if (firing && time_ms > pusherTimer_ms + pusherStallTime_ms) {
                            // Stall protection
                            pusherDriver.coast();
                            shotsToFire = 0;
                            firing = false;
                            flywheelState = STATE_IDLE;
                            DETTLAFF_DEBUG_PRINTF("Pusher motor stalled!\n");
                        }
                        break;
                        
                    case PUSHER_SOLENOID_OPENLOOP:
                        if (shotsToFire > 0 && !firing && time_ms > pusherTimer_ms + solenoidRetractTime_ms) {
                            // Extend solenoid
                            pusherDriver.drive(100.0f, pusherReverseDirection);
                            firing = true;
                            shotsToFire = MAX(0, shotsToFire - 1);
                            pusherTimer_ms = time_ms;
                            DETTLAFF_DEBUG_PRINTF("Solenoid extending, shots left: %d\n", shotsToFire);
                        } else if (firing && time_ms > pusherTimer_ms + solenoidExtendTime_ms) {
                            // Retract solenoid
                            pusherDriver.coast();
                            firing = false;
                            pusherTimer_ms = time_ms;
                            DETTLAFF_DEBUG_PRINTF("Solenoid retracting\n");
                        }
                        break;
                        
                    case NO_PUSHER:
                        break;
                }
            }
            break;
    }
    
    // 6. Motor control logic based on flywheelControl
    // Prevent division by zero if battery voltage is not available
    if (batteryVoltage_mv <= 0) {
        batteryVoltage_mv = 12000; // Default to 12V if no valid reading
        DETTLAFF_DEBUG_PRINTF("Warning: Using default battery voltage (12V) for calculations\n");
    }
    
    switch (flywheelControl) {
        case PID_CONTROL:
            for (int i = 0; i < 4; i++) {
                if (motorsPresent[i]) {
                    PIDError[i] = targetRPM[i] - motorRPM[i];
                    
                    PIDOutput[i] = KP * PIDError[i] + 
                                  KI * (PIDIntegral + PIDError[i] * loopTime_us / 1000000) + 
                                  KD * (PIDError[i] - PIDErrorPrior[i]) / loopTime_us * 1000000;
                    
                    closedLoopRPM[i] = PIDOutput[i] + motorRPM[i];
                    
                    // Calculate throttle as a proportion (0.0-1.0) based on target RPM
                    if (throttleValue[i] == 0) {
                        throttleValue[i] = MIN(1.0f, closedLoopRPM[i] / batteryVoltage_mv * 1000 / motorKv);
                    } else {
                        throttleValue[i] = MAX(MIN(1.0f, closedLoopRPM[i] / batteryVoltage_mv * 1000 / motorKv),
                                           throttleValue[i] - 0.001f);
                    }
                    
                    PIDErrorPrior[i] = PIDError[i];
                    PIDIntegral += PIDError[i] * loopTime_us / 1000000;
                }
            }
            break;
            
        case TWO_LEVEL_CONTROL:
            for (int i = 0; i < 4; i++) {
                if (motorsPresent[i]) {
                    if (targetRPM[i] == revRPM[i] && motorRPM[i] < fullThrottleRpmThreshold[i]) {
                        throttleValue[i] = 1.0f; // Full throttle when below threshold
                    } else {
                        throttleValue[i] = MAX(MIN(1.0f, targetRPM[i] / batteryVoltage_mv * 1000 / motorKv), 0.0f);
                    }
                }
            }
            break;
            
        case OPEN_LOOP_CONTROL:
            for (int i = 0; i < 4; i++) {
                if (motorsPresent[i]) {
                    throttleValue[i] = MAX(MIN(1.0f, targetRPM[i] / batteryVoltage_mv * 1000 / motorKv), 0.0f);
                }
            }
            break;
    }
    
    // 7. Update Motor Outputs
    for (int i = 0; i < 4; i++) {
        if (motorsPresent[i]) {
            // Set motor values directly as floats
            motor[i] = throttleValue[i];
        }
    }
    
    // Calculate loop time for next iteration
    loopTime_us = micros() - loopStartTimer_us;
}