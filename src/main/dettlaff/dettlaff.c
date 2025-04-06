/*
 * Dettlaff Blaster Control Module
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h> // Include standard I/O for printf

#include "platform.h"

// Include serial port definitions
#include "drivers/serial.h"
#include "io/serial.h"

#include "dettlaff.h"

// Include time-related functions
#include "common/time.h"

#include "drivers/system.h"

// Include beeper functions
#include "io/beeper.h"

void dettlaffInit(void)
{

    // Use standard printf for SITL console output, including the time
    printf("Dettlaff module initialized\n");
    fflush(stdout); // Ensure output is flushed immediately for SITL
}

void dettlaffUpdate(uint32_t currentTimeUs)
{
    // Main logic loop code goes here
    // For now, just print a message periodically (example)
    static uint32_t lastPrintTime = 0;
    if (currentTimeUs - lastPrintTime > 100000) { // print periodically
        printf("Dettlaff update running at %u us\n", currentTimeUs);
        lastPrintTime = currentTimeUs;
        fflush(stdout); // Ensure output is flushed immediately for SITL
    }

    // Keep parameter unused for now if no logic uses it yet
    // UNUSED(currentTimeUs); // Comment out UNUSED as currentTimeUs is now used in the print statement
}