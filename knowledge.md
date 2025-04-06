This project is to port the Dettlaff Flywheel Nerf Blaster Controller code from ESP32 Arduino to the STM32 Betaflight HA
The original ESP32 Arduino code is in the ESP32-Dettlaff directory
Please port over the ESP32-Dettlaff files to the new src/main/dettlaff directory, updating them to use the betaflight HAL and drivers, and dropping all wifi functionality.
You can test run the code by using `make TARGET=SITL` and then `./obj/main/betaflight_SITL.elf`

We are going to disable all betaflight flight controller code, meaning most files in the fc and flight directories. We want to keep the blackbox, configurator, and ESC passthrough functionality.
