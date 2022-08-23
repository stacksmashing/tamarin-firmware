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

### OpenOCD

To use Tamarin as a JTAG adapter you need to use our [OpenOCD fork](https://github.com/stacksmashing/openocd) that includes support for the Tamarin probe.

To enable JTAG on production iPhones they need to be demoted. For checkm8 vulnerable iPhones this can be done using [ipwndfu](https://github.com/axi0mX/ipwndfu).

Once the phone is successfully demoted the [bonobo configs](https://github.com/lambdaconcept/bonobo-configs/blob/master/t8015.cfg) can be used to connect to the iPhone like so:

```
openocd -f interface/tamarin.cfg -f t8015.cfg
```
