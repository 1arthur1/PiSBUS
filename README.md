# SBUS
Raspberry Pi SBUS library

# Description
I converted the SBUS library from [bolderflight](https://github.com/bolderflight/SBUS) to make it work on any Raspberry Pi.

Currently I tested it on a Raspberry Pi Zero W (with embedded Bluetooth/Wifi). I'm using it to send SBUS packet from the raspberry Pi to a flight controller (mine is the SP RACING F3 EVO). I was not able to test the read function because I don't have any receiver. I used the main serial port (/dev/ttyAMA0), don't forget to disable the serial console with raspi-config.

# Wiring
SBUS is very similar to UART (stop/start bits, parity etc) but its logic states are inverted.

If you are using CleanFlight you can simply type in CLI: "set sbus_inversion = OFF". If you are not using CleanFlight, you will need to invert the hardware signal. You can use 2 resistors and a transistor to do that.
