Important Instruction: After every file change, do a git commit that where the commit message starts with "cb: " which stands for codebuff and then a short description of the change.

This project is to port the Dettlaff Flywheel Nerf Blaster Controller code from ESP32 Arduino to the STM32 Betaflight HA
The original ESP32 Arduino code is in the Dettlaff directory, it's a read-only Github Submodule
The WiFi functionality will be dropped

## Motor Control Architecture

The Betaflight motor control stack has several layers:

1. Flight Controller (fc/core.c) - Highest level flight control
2. Mixer (flight/mixer.c) - Combines inputs and handles motor mixing
3. Motor Driver (drivers/motor.c) - Protocol-agnostic motor control
4. Protocol Layer (drivers/dshot.c) - Protocol-specific implementation
5. Hardware Layer - Direct hardware access

For direct motor control without flight controller complexity, use the Motor Driver layer which provides:
- Protocol-agnostic control
- Proper initialization/shutdown
- Simple normalized values (0.0-1.0)
- Key functions: motorDevInit(), motorWriteAll(), stopMotors()

## Parameter System

Parameters in Betaflight are organized into Parameter Groups (PGs):

1. Each PG needs:
   - Unique ID in pg_ids.h
   - Header file with struct definition and PG_DECLARE
   - Source file with PG_REGISTER and PG_RESET_TEMPLATE for defaults

2. Access parameters:
   - Read-only: const MyConfig_t *config = myConfig();
   - Write: MyConfig_t *config = myConfigMutable();
   - Only write during initialization

3. Parameters automatically appear in Configurator UI
   - No additional GUI code needed
   - Changes saved to EEPROM
   - For enum dropdowns, use uint8_t fields in the parameter group
   - Add comments showing the enum values and names for the configurator
   - Complex parameter groups can be split into multiple files (e.g. basic vs enum parameters)
   - Keep related parameters together - if parameters are tightly coupled (like enums and their associated values), they should be in the same parameter group

