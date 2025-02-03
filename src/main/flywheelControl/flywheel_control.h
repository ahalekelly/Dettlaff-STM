#ifndef FLYWHEEL_CONTROL_H
#define FLYWHEEL_CONTROL_H

#include <stdint.h>
#include "common/time.h"

// Initialize flywheel control (if any initialization is needed)
void flywheelControlInit(void);

// The main flywheel control loop which will be scheduled by the scheduler
// currentTimeUs is in microseconds.
void flywheelControlLoop(uint32_t currentTimeUs);

// Check if it's time to update the flywheel control
bool checkFlywheelControlUpdate(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);

#endif // FLYWHEEL_CONTROL_H
