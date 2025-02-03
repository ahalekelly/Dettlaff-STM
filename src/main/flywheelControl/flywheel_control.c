#include "platform.h"

#include "drivers/time.h"
#include "common/utils.h"
#include "config/feature.h"
#include "scheduler/scheduler.h"
#include "pg/dettlaff_params.h"
#include "drivers/motor.h"
#include "flywheelControl/flywheel_control.h"

#define USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"

// A very basic flywheel control loop:
// – Reads desired flywheel speed from our custom parameters (throttleValue, expected normalized 0.0–1.0)
// – Computes a command between motorConfig()->minthrottle and motorConfig()->maxthrottle
// – Writes the same command to all ESC outputs (thus preserving ESC passthrough)
FAST_CODE bool checkFlywheelControlUpdate(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    cliDebugPrintf("checkFlywheelControlUpdate running, time: %lu\n", currentTimeUs);
    UNUSED(currentDeltaTimeUs);
    static timeUs_t lastCalled = 0;
    const timeDelta_t interval = TASK_PERIOD_US(1000);
    
    if (currentTimeUs - lastCalled >= interval) {
        lastCalled = currentTimeUs;
        return true;
    }
    return false;
}

void flywheelControlInit(void) {
    cliDebugPrintf("flywheelControlInit running");
    rescheduleTask(TASK_FLYWHEEL, dettlaffParams()->flywheelControlFreq);
}

void flywheelControlLoop(uint32_t currentTimeUs) {
    UNUSED(currentTimeUs);
    cliDebugPrintf("checkFlywheelControlUpdate running, time: %lu\n", currentTimeUs);

    // Read the desired flywheel target (this parameter will be visible in the Configurator)
    float target = dettlaffParams()->throttleValue; // expected range: 0.0 to 1.0

    // Compute the desired motor output command (linear scaling)
    float minThrottle = motorConfig()->minthrottle;
    float maxThrottle = motorConfig()->maxthrottle;
    float command = minThrottle + target * (maxThrottle - minThrottle);

    // Build an array of motor commands.
    float motorValues[MAX_SUPPORTED_MOTORS] = {0};
    for (uint8_t i = 0; i < motorDeviceCount(); i++) {
        motorValues[i] = command;
    }

    // Pass the computed values to the motor driver.
    motorWriteAll(motorValues);
}
