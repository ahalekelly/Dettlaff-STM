#pragma once

#include "pg/pg.h"

// Ported Dettlaff enums from Dettlaff/src/types.h for integration with Betaflight.
// These enums are used in dettlaff_params.h for configurator dropdowns.

typedef enum {
    DETTLAFF_STATE_IDLE = 0,
    DETTLAFF_STATE_ACCELERATING = 1,
    DETTLAFF_STATE_FULLSPEED = 2,
} dettlaff_flywheelState_t;

typedef enum {
    DETTLAFF_NO_PUSHER = 0,
    DETTLAFF_PUSHER_MOTOR_CLOSEDLOOP = 1,
    DETTLAFF_PUSHER_SOLENOID_OPENLOOP = 2,
} dettlaff_pusherType_t;

typedef enum {
    DETTLAFF_NO_DRIVER = 0,
    DETTLAFF_FET_DRIVER = 1,
    DETTLAFF_DRV_DRIVER = 2,
    DETTLAFF_HBRIDGE_DRIVER = 3,
} dettlaff_pusherDriverType_t;

typedef enum {
    DETTLAFF_NO_SELECT_FIRE = 0,
    DETTLAFF_SWITCH_SELECT_FIRE = 1,
    DETTLAFF_BUTTON_SELECT_FIRE = 2,
} dettlaff_selectFireType_t;

typedef enum {
    DETTLAFF_OPEN_LOOP_CONTROL = 0,
    DETTLAFF_TWO_LEVEL_CONTROL = 1,
    DETTLAFF_PID_CONTROL = 2,
} dettlaff_flywheelControlType_t;

typedef enum {
    DETTLAFF_BURST_AUTO = 0,
    DETTLAFF_BURST_BURST = 1,
    DETTLAFF_BURST_BINARY = 2,
} dettlaff_burstFireType_t;

typedef enum {
    DETTLAFF_BOARD_V0_11 = 0,
    DETTLAFF_BOARD_V0_10,
    DETTLAFF_BOARD_V0_9,
    DETTLAFF_BOARD_V0_8,
    DETTLAFF_BOARD_V0_7,
    DETTLAFF_BOARD_V0_6,
    DETTLAFF_BOARD_V0_5,
    DETTLAFF_BOARD_V0_4_N20,
    DETTLAFF_BOARD_V0_4_NOID,
    DETTLAFF_BOARD_V0_3_N20,
    DETTLAFF_BOARD_V0_3_NOID,
    DETTLAFF_BOARD_V0_2,
    DETTLAFF_BOARD_V0_1
} dettlaff_board_t;

// All parameters are defined in dettlaff_params.h
