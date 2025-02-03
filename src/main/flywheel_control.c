#include "flywheel_control.h"
#include "drivers/motor.h"
#include "pg/dettlaff_params.h"  // Assume our custom parameters (like flywheel_target) are defined here

// A very basic flywheel control loop:
// – Reads desired flywheel speed from our custom parameters (flywheel_target, expected normalized 0.0–1.0)
// – Computes a command between motorConfig()->minthrottle and motorConfig()->maxthrottle
// – Writes the same command to all ESC outputs (thus preserving ESC passthrough)
void flywheelControlInit(void) {
    // Any one-time initialization of the flywheel controller
}

void flywheelControlLoop(uint32_t currentTimeUs) {
    // Read the desired flywheel target (this parameter will be visible in the Configurator)
    // (For example, if using a parameter group "dettlaff_params", assume there is a field "flywheel_target")
    float target = dettlaffParams()->flywheel_target; // expected range: 0.0 to 1.0

    // Compute the desired motor output command (linear scaling)
    float minThrottle = motorConfig()->minthrottle;
    float maxThrottle = motorConfig()->maxthrottle;
    float command = minThrottle + target * (maxThrottle - minThrottle);

    // Build an array of motor commands.
    // (For simplicity, assume our flywheel output uses the first motor; alternatively, copy to all if required)
    float motorValues[MAX_SUPPORTED_MOTORS] = {0};
    motorValues[0] = command;
    for (int i = 1; i < motorDeviceCount(); i++) {
        motorValues[i] = command;
    }

    // Pass the computed values to the motor driver.
    motorWriteAll(motorValues);

    // Optionally, record the output to Blackbox logging
    // (flywheel control outputs are recorded in the flywheel task's "I-frame" in our scheduler)
}
