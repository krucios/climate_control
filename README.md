# climate_control
The Raspberry Pi Pico project to track indoor air quality.

- Based on Pico SDK and BTStack
- Using CMake build system

## How to use this project
Make sure you have [Pico SDK](https://github.com/raspberrypi/pico-sdk) and [Pico extras](https://github.com/raspberrypi/pico-extras). Also, check that corresponding env variables are set correctly: `PICO_SDK_PATH` and `PICO_EXTRAS_PATH`. For tools required please refer to [Pico documentation](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html#raspberry-pi-pico-w).
1. Create a build directory from the project's root: `mkdir build && cd build`
2. Run CMake to generate Makefiles: `cmake -DPICO_BOARD=pico_w ../`
3. Build binarie: `make -j4`
4. Load them to Pico W: `sudo picotool load climate_control_sensor.uf2`
