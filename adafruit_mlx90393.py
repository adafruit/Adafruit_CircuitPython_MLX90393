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
    """
    def __init__(self, i2c_bus, address=0x0C):
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._command(_RESET)

    def _command(self, command):
        """
        Sends a single one-byte command to the device.
        :param command: The 8-bit command to send.
        """
        with self.i2c_device as i2c:
            # Pack data as unsigned char
            i2c.write(struct.pack('B', command))

    def _data(self):
        """
        Reads the two byte response to the previously issued command.
        """
        data = bytearray(2)
        while True:
            # While busy, the sensor doesn't respond to reads.
            try:
                with self.i2c_device as i2c:
                    i2c.readinto(data)
                    if data[0] != 0xff:  # Check if read succeeded.
                        break
            except OSError:
                pass
        # Unpack data as a pair of unsigned chars (status, value)
        value = struct.unpack('BB', data)
        return value

    def display_status(self, status):
        """
        Prints out the content of the status byte in a human-readble
        format.
        :param status: The 8-bit status byte to parse and print.
        """
        print("BURST Mode               : {}".format((status & (1 << 7)) >> 7))
        print("WOC Mode                 : {}".format((status & (1 << 6)) >> 6))
        print("SM Mode                  : {}".format((status & (1 << 5)) >> 5))
        print("Error                    : {}".format((status & (1 << 4)) >> 4))
        print("Single error detection   : {}".format((status & (1 << 3)) >> 3))
        print("Reset status             : {}".format((status & (1 << 2)) >> 2))
        print("Response bytes available : {}".format(status & 0b11))

    def reset(self):
        """
        Performs a software reset of the sensor.
        Returns the two byte response to the command, where the first
        byte is the status register.
        """
        self._command(_RESET)
        return self._data()
