# Tamarin Firmware
![Tamarin Logo](https://github.com/stacksmashing/tamarin-firmware/blob/main/media/tamarin-logo-300.png?raw=true)

## About this fork

This fork fixes various bugs in the Tamarin Firmware to make it actually usable. However, it currently does not support JTAG/SWD, only UART, Reset and Reset to DFU.

UART is enabled automatically when a device is connected.

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

- [GP0] L1n (Purple)
- [GP1] L1p (Orange)
- [GND] GND (Black)
- [GP2] ID1 (Blue)
- [GP3] ID0 (Yellow)
- [5V]  5V  (Red)

Note: The colors might be different for your cable. I recommend checking the pinout using a voltmeter.

![Lightning](https://github.com/stacksmashing/tamarin-firmware/blob/main/media/lightning.png?raw=true)

Another cable was observed to have the following pinout:

- [GP0] L1n (Green)
- [GP1] L1p (White)
- [GND] GND (Black)
- [GP2] ID1 (Orange)
- [GP3] ID0 (Red)
- [5V]  5V  (Yellow)

If you would like to connect to your device over USB, cut a USB cable and connect its wires like this:
- USB cable D+ (green)  -> L0p (color depends on your cable)
- USB cable D- (white)  -> L0n (color depends on your cable)
- USB cable GND (black) -> GND

## Usage

Tamarin Cable provides three USB endpoints, of which two are serial ports.

Serial port 1 is the control serial port, use it to configure DCSD/JTAG mode.

Serial port 2 is the DCSD port, when Tamarin Cable is in DCSD mode the serial output will be provided here.

### OpenOCD

To use Tamarin as a JTAG adapter you need to use our [OpenOCD fork](https://github.com/stacksmashing/openocd) that includes support for the Tamarin probe.

To enable JTAG on production iPhones they need to be demoted. For checkm8 vulnerable iPhones this can be done using [ipwndfu](https://github.com/axi0mX/ipwndfu).

Once the phone is successfully demoted the [bonobo configs](https://github.com/lambdaconcept/bonobo-configs/blob/master/t8015.cfg) can be used to connect to the iPhone like so:

```
openocd -f interface/tamarin.cfg -f t8015.cfg
```
