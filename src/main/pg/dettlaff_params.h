#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pg/pg.h"
#include "dettlaff.h"

typedef struct dettlaffParams_s {
    // Motor Settings
    uint16_t motorSpeedDefault;     // Default motor speed (0-2000)
    uint16_t motorSpeedMax;         // Maximum allowed motor speed
    uint8_t  motorCount;            // Number of motors (1-8)
    uint8_t  reverseMotorDir;       // Reverse motor direction (0/1)

    // Flywheel Settings
    uint8_t  variableFPS;               // true = 1, false = 0
    int32_t  revRPMset[3][4];           // Firing mode groups: each row has 4 motor RPM settings
    uint32_t idleTimeSet_ms[3];         // Idle times (ms) for each firing mode
    uint32_t firingDelaySet_ms[3];      // Firing delay (ms) before dart fires (non-idle)
    uint32_t firingDelayIdleSet_ms[3];  // Firing delay (ms) when starting from idle
    uint32_t spindownSpeed;             // RPM per ms reduction

    // Motor Control Settings
    int32_t  motorKv;                   // Motor velocity constant
    int32_t  idleRPM[4];               // Idle RPM for each motor
    uint8_t  dshotMode;                // Dshot mode (0=DSHOT150, 1=DSHOT300, 2=DSHOT600, 3=OFF)
    uint8_t  brushedFlywheels;         // 1 = brushed, 0 = brushless

    // Closed Loop Settings
    uint8_t  motors[4];                 // Which motors are hooked up (1=true, 0=false)
    uint8_t  timeOverrideWhenIdling;    // 1 = true, 0 = false
    int32_t  fullThrottleRpmTolerance;  // Tolerance for full throttle override
    int32_t  firingRPMTolerance;        // Tolerance for firing dart
    int32_t  minFiringRPM;              // Minimum RPM for firing mode
    int32_t  minFiringDelaySet_ms[3];   // Minimum delay (ms) before fire (non-idle)
    int32_t  minFiringDelayIdleSet_ms[3]; // Minimum delay (ms) before fire when idling

    // Select Fire Settings
    uint32_t burstLengthSet[3];         // Dart burst lengths for each mode
    uint8_t  burstModeSet[3];           // Burst mode: 0=AUTO, 1=BURST, 2=BINARY
    uint32_t binaryTriggerTimeout_ms;   // Timeout (ms) for binary trigger
    uint8_t  defaultFiringMode;         // Default mode for select-fire

    // Mode Selection Enums (appear as dropdowns in configurator)
    uint8_t  flywheelState;         // dettlaff_flywheelState_t (0=IDLE, 1=ACCELERATING, 2=FULLSPEED)
    uint8_t  pusherType;            // dettlaff_pusherType_t (0=NO_PUSHER, 1=MOTOR_CLOSEDLOOP, 2=SOLENOID_OPENLOOP)
    uint8_t  pusherDriverType;      // dettlaff_pusherDriverType_t (0=NO_DRIVER, 1=FET_DRIVER, 2=DRV_DRIVER, 3=HBRIDGE_DRIVER)
    uint8_t  selectFireType;        // dettlaff_selectFireType_t (0=NO_SELECT_FIRE, 1=SWITCH_SELECT_FIRE, 2=BUTTON_SELECT_FIRE)
    uint8_t  flywheelControlType;   // dettlaff_flywheelControlType_t (0=OPEN_LOOP_CONTROL, 1=TWO_LEVEL_CONTROL, 2=PID_CONTROL)
    uint8_t  burstFireType;         // dettlaff_burstFireType_t (0=AUTO, 1=BURST, 2=BINARY)

    // System Settings
    uint8_t  printTelemetry;            // Enable telemetry (1=yes, 0=no)
    uint32_t lowVoltageCutoff_mv;       // Battery cutoff voltage (mV)
    float    voltageCalibrationFactor;  // Battery voltage calibration factor

    // Input Pins (set to 0 if not used)
    uint8_t triggerSwitchPin;
    uint8_t revSwitchPin;
    uint8_t cycleSwitchPin;
    uint8_t select0Pin;
    uint8_t select1Pin;
    uint8_t select2Pin;

    // Board Settings (appears as dropdown in configurator)
    uint8_t board;                  // dettlaff_board_t (0=V0_11, 1=V0_10, etc.)

    // Pusher Settings
    uint32_t pusherVoltage_mv;        // Voltage threshold for pusher PWM (mV)
    uint8_t  pusherReverseDirection;  // 1 = reverse, 0 = normal

    // Solenoid Settings
    uint16_t solenoidExtendTime_ms;   // Extend time (ms)
    uint16_t solenoidRetractTime_ms;  // Retract time (ms)

    // Pusher Motor Settings
    uint32_t pusherReverseBrakingVoltage_mv; // Braking voltage (mV)
    uint8_t  pusherReversePolarityDuration_ms; // Reverse polarity duration (ms)
    uint32_t pusherDwellTime_ms;       // Dwell time at end of cycle (ms)
    uint8_t  pusherBrakeOnDwell;       // 1 = brake during dwell, 0 = coast

    // Advanced Settings
    uint16_t pusherStallTime_ms;       // Stall detection time (ms)
    uint8_t  revSwitchNormallyClosed;  // 1 = normally closed, 0 = open
    uint8_t  triggerSwitchNormallyClosed; // 1 = normally closed, 0 = open
    uint8_t  cycleSwitchNormallyClosed;   // 1 = normally closed, 0 = open
    uint16_t debounceTime_ms;          // Debounce time for switches (ms)
    uint16_t pusherDebounceTime_ms;    // Debounce time for pusher (ms)
    int      voltageAveragingWindow;   // Voltage averaging window size
    uint32_t pusherCurrentSmoothingFactor; // Smoothing factor for current measurement
    uint8_t  telemetryInterval_ms;     // Telemetry update interval (ms)
    uint16_t targetLoopTime_us;        // Main loop target period (µs)
    float    maxDutyCycle_pct;         // Maximum duty cycle (%)
    uint8_t  deadtime;                 // PWM dead time (µs)
    uint16_t pwmFreq_hz;               // PWM frequency (Hz)
    uint16_t servoFreq_hz;             // Servo PWM frequency (Hz)
    uint8_t  pusherReverseOnOverrun;   // 1 = reverse on overrun, 0 = not used
    uint8_t  pusherEndReverseBrakingEarly; // 1 = end braking early, 0 = not used

    // PID Settings
    float    KP;
    float    KI;
    float    KD;
} dettlaffParams_t;

PG_DECLARE(dettlaffParams_t, dettlaffParams);
