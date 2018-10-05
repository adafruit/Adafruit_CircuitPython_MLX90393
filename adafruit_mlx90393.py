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
from array import array

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MLX90393.git"

_CMD_SB = const(0b00010000)   # Start burst mode
_CMD_SW = const(0b00100000)   # Start wakeup on change mode
_CMD_SM = const(0b00110000)   # Start single-measurement mode
_CMD_RM = const(0b01000000)   # Read measurement
_CMD_RR = const(0b01010000)   # Read register
_CMD_WR = const(0b01100000)   # Write register
_CMD_EX = const(0b10000000)   # Exit mode
_CMD_HR = const(0b11010000)   # Memory recall
_CMD_HS = const(0b11100000)   # Memory store
_CMD_RT = const(0b11110000)   # Reset
_CMD_NOP = const(0x00)        # NOP

_CMD_AXIS_ALL = const(0xE)    # X+Y+Z axis bits for commands

_CMD_REG_CONF1 = const(0x00)  # Gain
_CMD_REG_CONF2 = const(0x01)  # Burst, comm mode
_CMD_REG_CONF3 = const(0x02)  # Oversampling, Filter, Resolution
_CMD_REG_CONF4 = const(0x03)  # Sensitivity drift

_GAIN_5X = const(0x0)
_GAIN_4X = const(0x1)
_GAIN_3X = const(0x2)
_GAIN_2_5X = const(0x3)
_GAIN_2X = const(0x4)
_GAIN_1_67X = const(0x5)
_GAIN_1_33X = const(0x6)
_GAIN_1X = const(0x7)
_GAIN_SHIFT = const(4)

_RES_2_15 = const(0)   # +/- 2^15
_RES_2_15B = const(1)  # +/- 2^15
_RES_22000 = const(2)  # +/- 22000
_RES_11000 = const(3)  # +/- 11000
_RES_SHIFT = const(5)

_HALLCONF = const(0x0C)     # Hall plate spinning rate adjust.


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
        self._status_last = 0
        self._gain_current = _GAIN_1_67X
        self._res_current = _RES_2_15
        # The lookup table below allows you to convert raw sensor data to uT
        # using the approrpiate (gain/resolution-dependant) lsb-per-uT
        # coefficient below. Note that the W axis has a different coefficient
        # than the x and y axis.
        self._lsb_lookup = [
            # 5x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.805, 1.468], [1.610, 2.936], [3.220, 5.872], [6.440, 11.744]],
            # 4x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.644, 1.174], [1.288, 2.349], [2.576, 4.698], [5.152, 9.395]],
            # 3x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.483, 0.881], [0.966, 1.762], [1.932, 3.523], [3.864, 7.046]],
            # 2.5x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.403, 0.734], [0.805, 1.468], [1.610, 2.936], [3.220, 5.872]],
            # 2x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.322, 0.587], [0.644, 1.174], [1.288, 2.349], [2.576, 4.698]],
            # 1.667x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.268, 0.489], [0.537, 0.979], [1.073, 1.957], [2.147, 3.915]],
            # 1.333x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.215, 0.391], [0.429, 0.783], [0.859, 1.566], [1.717, 3.132]],
            # 1x gain: res0[xy][z], res1[xy][z], res2[xy][z], res3[xy][z]
            [[0.161, 0.294], [0.322, 0.587], [0.644, 1.174], [1.288, 2.349]]
        ]
        # Put the device in a known state to start
        self.reset()
        # Set gain to 1.667x by default */
        self._transceive(bytes([0x60,
                               0x00,
                               self._gain_current << _GAIN_SHIFT | _HALLCONF,
                               0x00]))

    def _transceive(self, payload, len=0):
        """
        Writes the specified 'payload' to the sensor
        Returns the results of the write attempt.
        :param bytearray payload: The byte array to write to the sensor
        :param len: (optional) The numbers of bytes to read back (default=3)
        """
        # Write 'value' to the specified register
        with self.i2c_device as i2c:
            i2c.write(payload)

        # Read the response (+1 to account for the mandatory status byte!)
        data = bytearray(len+1)
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
        # Track status byte
        _status_last = data[0]
        # Unpack data (status byte, big-endian 16-bit register value)
        if self._debug:
            print("\t[{}]".format(time.monotonic()))
            print("\t Writing :", [hex(b) for b in payload])
            print("\tResponse :", [hex(b) for b in data])
            print("\t  Status :", hex(data[0]))
        return data

    @property
    def last_status(self):
        """
        Returns the last status byte received from the sensor.
        """
        return self._status_last

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
        """
        if (self._debug):
            print("Resetting sensor")
        time.sleep(2)
        _status_last = self._transceive(bytes([_CMD_RT]))
        return _status_last

    def read_data(self, delay=0.0, raw=False):
        """
        Reads a single X/Y/Z sample from the magnetometer.
        """
        # Set the device to single measurement mode
        self._transceive(bytes([_CMD_SM | _CMD_AXIS_ALL]))
        # Wait a bit
        time.sleep(delay)
        # Read 6 bytes back from 0x4E
        data = self._transceive(bytes([_CMD_RM | _CMD_AXIS_ALL]), 6)
        # Parse the data (status byte, 3 * signed 16-bit integers)
        self._status_last, x, y, z = struct.unpack(">Bhhh", data)

        if raw:
            # Return the raw int values if requested
            return x, y, z
        else:
            # Convert the units to uT based on gain and resolution
            x *= self._lsb_lookup[self._gain_current][self._res_current][0]
            y *= self._lsb_lookup[self._gain_current][self._res_current][0]
            z *= self._lsb_lookup[self._gain_current][self._res_current][1]
            return x, y, z
