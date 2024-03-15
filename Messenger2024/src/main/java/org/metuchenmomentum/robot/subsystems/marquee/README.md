# The Marquee Control Library

Welcome to the Marquee Control Library, which is used to send commands
from the RoboRio to the ESP32 that controls the 8 x 32 WS2812B
individually addressable LED array. 

## Overview

The library provides four classes that sends commands via an SPI connection
from the RoboRio to the ESP32. The library provides the following
classes:

* `DisplayCommand`, an `enum` of supported display functions, ranging
  from single pixel display to a full text crawl
* `DisplayMessage`, the messsage that the RoboRio sends to the ESP32.
  The message includes foreground and background colors, text to display,
  and other information. Please see the class documentation for details.
* `DisplayConnection`, which specifies the message sending API.
* `SpiDisplayConnection`, a `DisplayConnection` implementation that
  sends messages via a
  [Serial Peripheral Interface](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface)
  (SPI) connection.

## Using the API

An Arduino IDE is needed to install the Marquee software on the ESP32.
We have used
[Sloeber](https://eclipse.baeyens.it/),
an
[Eclipse](https://www.eclipse.org/)-based
IDE. The standard
[Arduino](https://www.arduino.cc/)
[IDE](https://www.arduino.cc/en/software)
will install the software, but the project is too large to use it for
serious development. There are probably other candidates. Use the one
that suits you. Whichever you choose, be sure to install the ESP32
Arduino libraries and to generate code for the ESP32 S2.

To use the api:

1. Install an Arduino IDE onto 
2. Wire the ESP32 to the Display Panel as described in the
   [Marquee Library Documentation](https://github.com/frc-team7587/Marquee-1/blob/main/TestLedMatrix/README.md#wiring-the-display-panel).
3. Wire the RoboRio to the ESP32 as described in the
   [Marquee Library Documentation](https://github.com/frc-team7587/Marquee-1/blob/main/TestLedMatrix/README.md#wiring-the-display-panel).
   Be sure to wire the 5 Volt and 3.3 Volt buck converters as described.
   **DO NOT MIX THEM UP** as you will destroy the ESP32 if you do.
4. Incorporate the Marquee library into the robot control software.
   Invoke the API as desired.
5. Enjoy the fruits of your labor.

## Classes

The library provides the following classes that work together to drive
the marquee

### `DisplayCommand`

The `DisplayCommand` enumeration specifies the Marquee's supported
actions. Actions range from moving a single pixel around the array,
features that mainly support hardware level debugging, to a full fledged
text crawl.

:warning: keep the `DisplayCommand` enumeration in sync with the
C++ enum specified in Marquee library's
[`DisplayCommand.h`](https://github.com/frc-team7587/Marquee-1/blob/main/TestLedMatrix/DisplayCommand.h)
file. If you change one, you **must** make the corresponding change in the other.

### `DisplayConnection`

The `DisplayConnection` interface specifies the API that sends messages
to the ESP32. It contains one function that sends a command.

### `DisplayMessage`

The `DisplayMessage` class simplifies command generation. Pt provides
convenient defaults for all fields, zero for numeric values and the
empty string for text values, and allows the users to set:

* Text to display, if any
* Display command, what the marquee should do with the provided
  values
* Foreground Red, Green, and Blue, the components of the foreground
  color. Note that text is displayed in the foreground color.
* Background Red, Green, and Blue, the components of the background
  color.
* Delay 1 and Delay 2, delays to apply to the display. Delay meanings
  vary by command. For scrolling text, delay 1 specifies the dwell time
  for the initial display, and delay 2 specifies the interval between
  crawl shifting. Note that delays are in ESP32 time ticks, where a
  tick is one millisecond.

:warning: the LED matrix consumes 75 watts when run at full power,equivalent to 750 watts incandescent. Running at full power
significantly shortens LED lifetime. Select RGB intensities
accordingly.

### `SpiDisplayConnection`

The `SpiDisplayConnection` class is a
[`DisplayConnection`](#displayconnection)
implementation that sends messages over
[SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface).
The channel is (i.e. slave select pin) is set at construction.

###`UsbDisplayConnection`

The `UsbDisplayConnection` class is a 
[`DisplayConnection`](#displayconnection) 
implementation that sends commands over a USB serial connection,
a USB connection that emulates a asycronous serial terminal,
a.k.a. a serial port. 

### `UsbSerialDeviceList`

The `UsbSerialDeviceList` finds all serial USB devices, file
paths of the form `/dev/ttyUSBn`, where `n` is 0, 1, or 2.

### `UsbSerialPortBuilder`

The builds `SerialPort` instances that connect to
USB serial devices which are file
paths of the form `/dev/ttyUSBn`, where `n` is 0, 1, or 2. Note that
the `SerialPort` class is provided by the WPI library. 

### VacuousDisplayConnection

The `VacuousDisplayConnection` is a `DisplayConnection` implementation
that does absolutely nothing, which is used when a connection to a
marquee cannot be found. It allows application code to send messages
to a marquee when no physical connection exists.
