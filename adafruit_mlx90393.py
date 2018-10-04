# The MIT License (MIT)
#
# Copyright (c) 2018 Kevin Townsend for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
`adafruit_mlx90393`
====================================================
This is a breakout for the Adafruit MLX90393 magnetometer sensor breakout.
* Author(s): ktown
Implementation Notes
--------------------
**Hardware:**
* Adafruit `MLX90393 Magnetometer Sensor Breakout Board
  <https://www.adafruit.com/product/XXXX>`_ (Product ID: XXXX)
**Software and Dependencies:**
* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""
import time
try:
    import struct
except ImportError:
    import ustruct as struct

from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MLX90393.git"

_RESET = const(0x0F)


class MLX90393:
    """
    Driver for the MLX90393 magnetometer.
    :param i2c_bus: The `busio.I2C` object to use. This is the only
    required parameter.
    :param int address: (optional) The I2C address of the device.
    :param bool debug: (optional) Enable debug output.
    """
    def __init__(self, i2c_bus, address=0x0C, debug=False):
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._debug = debug
        self.reset()
        # Write config data
        self._transceive(bytes([0x60, 0x02, 0xB4, 0x08]))

    def _command(self, reg):
        """
        Sends a command to sensor using the specified register.

        Returns two values. The first is the 8-bit status value, and the second
        is the 16-bit value returned from the register itself.

        :param reg: The 8-bit register to read
        """
        with self.i2c_device as i2c:
            i2c.write(bytes([reg & 0xFF]))

        # Read the response back
        data = bytearray(3)
        while True:
            # While busy, the sensor doesn't respond to reads.
            try:
                with self.i2c_device as i2c:
                    i2c.readinto(data)
                    # Make sure we have something in the response
                    if data[0]:
                        break
            except OSError:
                pass
        # Unpack data (status byte, big-endian 16-bit register value)
        value = struct.unpack('>BH', data)
        if self._debug:
            print("\t[{}]".format(time.monotonic()))
            print("\t Command :", hex(reg & 0xFF))
            print("\tResponse :", [hex(b) for b in data])
            print("\t  Status :", hex(value[0]))
            print("\t   Value :", value[1])
        return value

    def _transceive(self, payload, len=3):
        """
        Writes the specified 'payload' to the sensor
        Returns the results of the write attempt.
        :param bytearray payload: The byte array to write to the sensor
        :param len: The numbers of bytes to read back (default=3)
        """
        # Write 'value' to the specified register
        with self.i2c_device as i2c:
            i2c.write(payload)

        # Read the response
        data = bytearray(len)
        while True:
            # While busy, the sensor doesn't respond to reads.
            try:
                with self.i2c_device as i2c:
                    i2c.readinto(data)
                    # Make sure we have something in the response
                    if data[0]:
                        break
            except OSError:
                pass
        # Unpack data (status byte, big-endian 16-bit register value)
        if self._debug:
            print("\t[{}]".format(time.monotonic()))
            print("\t Writing :", [hex(b) for b in payload])
            print("\tResponse :", [hex(b) for b in data])
            print("\t  Status :", hex(data[0]))
        return data

    def display_status(self, status):
        """
        Prints out the content of the status byte in a human-readble
        format.
        :param status: The 8-bit status byte to parse and print.
        """
        print("BURST Mode               :", (status & (1 << 7)) != 0)
        print("WOC Mode                 :", (status & (1 << 6)) != 0)
        print("SM Mode                  :", (status & (1 << 5)) != 0)
        print("Error                    :", (status & (1 << 4)) != 0)
        print("Single error detection   :", (status & (1 << 3)) != 0)
        print("Reset status             :", (status & (1 << 2)) != 0)
        print("Response bytes available :", status & 0b11)

    def reset(self):
        """
        Performs a software reset of the sensor.
        Returns the two byte response to the command, where the first
        byte is the status register.
        """
        if (self._debug):
            print("Resetting sensor")
        time.sleep(2)
        return self._command(_RESET)

    def read_data(self):
        """
        Reads a single X/Y/Z sample from the magnetometer.
        """
        # Set the device to single measurement mode
        self._transceive(bytes([0x3E]))
        # Wait a bit
        time.sleep(0.5)
        # Read 7 bytes back from 0x4E
        data = self._transceive(bytes([0x4E]), 7)
        # Parse the data (status byte, 3 * signed 16-bit integers)
        status, x, y, z = struct.unpack(">Bhhh", data)
        return status, x, y, z
