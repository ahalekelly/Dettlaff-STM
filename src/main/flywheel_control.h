#ifndef FLYWHEEL_CONTROL_H
#define FLYWHEEL_CONTROL_H

#include <stdint.h>

// Initialize flywheel control (if any initialization is needed)
void flywheelControlInit(void);

// The main flywheel control loop which will be scheduled by the scheduler
// currentTimeUs is in microseconds.
void flywheelControlLoop(uint32_t currentTimeUs);

#endif // FLYWHEEL_CONTROL_H
