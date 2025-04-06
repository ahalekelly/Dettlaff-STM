/*
 * Dettlaff Blaster Control Module
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

// Dettlaff - Function prototypes
void dettlaffInit(void);
void dettlaffUpdate(uint32_t currentTimeUs); // Change return type to void