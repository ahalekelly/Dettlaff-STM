This project is to port the Dettlaff Flywheel Nerf Blaster Controller code from ESP32 Arduino to the STM32 Betaflight HA
The original ESP32 Arduino code is in the Dettlaff directory, it's a read-only Github Submodule
The Dettlaff WiFi functionality will be dropped

We are going to disable all betaflight flight controller code, meaning most files in the fc and flight directories. We want to keep the blackbox, configurator, and ESC passthrough functionality.

We want to start with a hello world, please create a new file called dettlaff where we will put our blaster control code.

You can run the code by using `make TARGET=SITL` and then `./obj/main/betaflight_SITL.elf`