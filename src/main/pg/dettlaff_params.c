#include "platform.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "dettlaff_params.h"

PG_REGISTER_WITH_RESET_TEMPLATE(dettlaffParams_t, dettlaffParams, PG_DETTLAFF_PARAMS, 1);

PG_RESET_TEMPLATE(dettlaffParams_t, dettlaffParams,
    // Motor Settings
    .motorSpeedDefault = 1000,      // 50% speed by default
    .motorSpeedMax = 2000,          // Maximum motor speed
    .motorCount = 2,                // Default to 2 motors
    .reverseMotorDir = 0,           // Normal motor direction

    // Flywheel Settings
    .variableFPS = 1,
    .revRPMset = { { 40000, 40000, 40000, 40000 }, { 25000, 25000, 25000, 25000 }, { 14000, 14000, 14000, 14000 } },
    .idleTimeSet_ms = { 30000, 5000, 2000 },
    .firingDelaySet_ms = { 150, 125, 100 },
    .firingDelayIdleSet_ms = { 125, 100, 80 },
    .spindownSpeed = 30,

    // Motor Control Settings
    .motorKv = 3200,
    .idleRPM = { 500, 500, 500, 500 },
    .dshotMode = 1,               // DSHOT300
    .brushedFlywheels = 0,

    // Closed Loop Settings
    .motors = { 1, 1, 1, 1 },
    .timeOverrideWhenIdling = 1,
    .fullThrottleRpmTolerance = 5000,
    .firingRPMTolerance = 10000,
    .minFiringRPM = 10000,
    .minFiringDelaySet_ms = { 0, 0, 0 },
    .minFiringDelayIdleSet_ms = { 0, 0, 0 },

    // Select Fire Settings
    .burstLengthSet = { 100, 5, 1 },
    .burstModeSet = { 0, 0, 1 },  // 0=AUTO, 1=BURST, 2=BINARY
    .binaryTriggerTimeout_ms = 2000,
    .defaultFiringMode = 1,

    // Mode Selection Enums
    .flywheelState = DETTLAFF_STATE_IDLE,
    .pusherType = DETTLAFF_NO_PUSHER,
    .pusherDriverType = DETTLAFF_NO_DRIVER,
    .selectFireType = DETTLAFF_NO_SELECT_FIRE,
    .flywheelControlType = DETTLAFF_OPEN_LOOP_CONTROL,
    .burstFireType = DETTLAFF_BURST_AUTO,

    // System Settings
    .printTelemetry = 0,
    .lowVoltageCutoff_mv = 10000,  // 2500*4
    .voltageCalibrationFactor = 1.0f,

    // Input Pins
    .triggerSwitchPin = 32,
    .revSwitchPin = 15,
    .cycleSwitchPin = 23,
    .select0Pin = 25,
    .select1Pin = 0,
    .select2Pin = 33,

    // Board Settings
    .board = DETTLAFF_BOARD_V0_9,  // Default to v0.9

    // Pusher Settings
    .pusherVoltage_mv = 13000,
    .pusherReverseDirection = 0,

    // Solenoid Settings
    .solenoidExtendTime_ms = 20,
    .solenoidRetractTime_ms = 35,

    // Pusher Motor Settings
    .pusherReverseBrakingVoltage_mv = 16000,
    .pusherReversePolarityDuration_ms = 5,
    .pusherDwellTime_ms = 0,
    .pusherBrakeOnDwell = 0,

    // Advanced Settings
    .pusherStallTime_ms = 750,
    .revSwitchNormallyClosed = 0,
    .triggerSwitchNormallyClosed = 0,
    .cycleSwitchNormallyClosed = 0,
    .debounceTime_ms = 100,
    .pusherDebounceTime_ms = 25,
    .voltageAveragingWindow = 1,
    .pusherCurrentSmoothingFactor = 90,
    .telemetryInterval_ms = 5,
    .targetLoopTime_us = 1000,
    .maxDutyCycle_pct = 98.0f,
    .deadtime = 10,
    .pwmFreq_hz = 20000,
    .servoFreq_hz = 200,
    .pusherReverseOnOverrun = 0,
    .pusherEndReverseBrakingEarly = 0,

    // PID Settings
    .KP = 1.5f,
    .KI = 0.0f,
    .KD = 0.5f
);
