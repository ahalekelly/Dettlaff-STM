This project is to port the Dettlaff Flywheel Nerf Blaster Controller code from ESP32 Arduino to the STM32 Betaflight HA
The original ESP32 Arduino code is in the ESP32-Dettlaff directory
Please port over the ESP32-Dettlaff files to the new src/main/dettlaff directory, updating them to use the betaflight HAL and drivers, dropping all wifi functionality. We're targeting the 
We're targeting the STM32F405 (stm32f4xx) platform, use that when looking at drivers for reference.
Please ask me if you have any uncertainties about this, and lay out your architecture and plan first so that I can approve it.
You can test run the code by using `make TARGET=SITL` and then `./obj/main/betaflight_SITL.elf`

We are going to disable all betaflight flight controller code, meaning most files in the fc and flight directories. We want to keep the blackbox, configurator, and ESC passthrough functionality.
