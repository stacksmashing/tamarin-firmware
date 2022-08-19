# Tamarin Firmware
![Tamarin Logo](https://github.com/stacksmashing/tamarin-firmware/blob/main/media/tamarin-logo-300.png?raw=true)

## Build

```
mkdir build
cd build
cmake ..
make
```

## Hooking it up

![Pinout diagram](https://github.com/stacksmashing/tamarin-firmware/blob/main/media/pinout.png?raw=true)

With this cable, connect: 

- Purple to GPIO1
- Orange to GPIO2
- Black to any GND pin
- Blue to GPIO3
- Yellow to GPIO4
- Red to 5V

## Usage

Tamarin Cable provides three USB endpoints, of which two are serial ports.

Serial port 1 is the control serial port, use it to configure DCSD/JTAG mode.

Serial port 2 is the DCSD port, when Tamarin Cable is in DCSD mode the serial output will be provided here.
