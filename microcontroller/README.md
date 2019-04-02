# Microcontroller code

## Overview

This subdirectory contains code that implements an example serial application on a microcontroller, useful to talk with ros2_to_serial_bridge.

## Hardware

At the moment, this code has only been ported to the STM32F3Discovery board.  It should be pretty straightforward to port to other Cortex-M parts (especially those supported by libopencm3).

## Components

There are 5 main components in the microcontroller code:

* [libopencm3](https://github.com/libopencm3/libopencm3) - The low-level drivers for the Cortex-M processor used in this project.  There is a vendored version of the library here; see the [libopencm3/README](README) for more details.  LGPLv3 license.

* [freertos](https://www.freertos.org/) - The RTOS used in this project.  There is a vendored version of the RTOS here; see the [freertos/README](README) for more details.  MIT license.

* [Micro-CDR](https://github.com/eProsima/Micro-CDR) - The CDR serialization/deserialization used in this project.  There is a vendored version of the library here; see the [microCDR/README](README) for more details.  Apache v2 license.

* The "library" to implement a minimal ROS-like API.  Still in development.  Apache v2 license.

* The top-level application/main.  Apache v2 license.

## Setup

To build this project, a few dependencies need to be installed.  The instructions below have been tested on Ubuntu 16.04 and 18.04, but it should work fine on other platforms if the `arm-none-eabi` compiler and tools, and openocd, are available.

```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded openocd
```

## Building

Once the dependencies are installed, the project can be built with:

```
$ make
```

And it can be flashed to the board with:

```
$ make flash
```
