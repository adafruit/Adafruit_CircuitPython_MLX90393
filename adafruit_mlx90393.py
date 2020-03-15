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
  <https://www.adafruit.com/product/4022>`_ (Product ID: 4022)

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

_CMD_SB = const(0b00010000)  # Start burst mode
_CMD_SW = const(0b00100000)  # Start wakeup on change mode
_CMD_SM = const(0b00110000)  # Start single-measurement mode
_CMD_RM = const(0b01000000)  # Read measurement
_CMD_RR = const(0b01010000)  # Read register
_CMD_WR = const(0b01100000)  # Write register
_CMD_EX = const(0b10000000)  # Exit mode
_CMD_HR = const(0b11010000)  # Memory recall
_CMD_HS = const(0b11100000)  # Memory store
_CMD_RT = const(0b11110000)  # Reset
_CMD_NOP = const(0x00)  # NOP

_CMD_AXIS_ALL = const(0xE)  # X+Y+Z axis bits for commands

_CMD_REG_CONF1 = const(0x00)  # Gain
_CMD_REG_CONF2 = const(0x01)  # Burst, comm mode
_CMD_REG_CONF3 = const(0x02)  # Oversampling, Filter, Resolution
_CMD_REG_CONF4 = const(0x03)  # Sensitivity drift

GAIN_5X = 0x0
GAIN_4X = 0x1
GAIN_3X = 0x2
GAIN_2_5X = 0x3
GAIN_2X = 0x4
GAIN_1_67X = 0x5
GAIN_1_33X = 0x6
GAIN_1X = 0x7
_GAIN_SHIFT = const(4)

_RES_2_15 = const(0)  # +/- 2^15
_RES_2_15B = const(1)  # +/- 2^15
_RES_22000 = const(2)  # +/- 22000
_RES_11000 = const(3)  # +/- 11000
_RES_SHIFT = const(5)

_HALLCONF = const(0x0C)  # Hall plate spinning rate adjust.

STATUS_OK = 0x3

# The lookup table below allows you to convert raw sensor data to uT
# using the appropriate (gain/resolution-dependant) lsb-per-uT
# coefficient below. Note that the z axis has a different coefficient
# than the x and y axis. Assumes HALLCONF = 0x0C!
_LSB_LOOKUP = (
    # 5x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.751, 1.210), (1.502, 2.420), (3.004, 4.840), (6.009, 9.680)),
    # 4x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.601, 0.968), (1.202, 1.936), (2.403, 3.872), (4.840, 7.744)),
    # 3x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.451, 0.726), (0.901, 1.452), (1.803, 2.904), (3.605, 5.808)),
    # 2.5x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.376, 0.605), (0.751, 1.210), (1.502, 2.420), (3.004, 4.840)),
    # 2x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.300, 0.484), (0.601, 0.968), (1.202, 1.936), (2.403, 3.872)),
    # 1.667x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.250, 0.403), (0.501, 0.807), (1.001, 1.613), (2.003, 3.227)),
    # 1.333x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.200, 0.323), (0.401, 0.645), (0.801, 1.291), (1.602, 2.581)),
    # 1x gain: res0(xy)(z), res1(xy)(z), res2(xy)(z), res3(xy)(z)
    ((0.150, 0.242), (0.300, 0.484), (0.601, 0.968), (1.202, 1.936)),
)


class MLX90393:
    """
    Driver for the MLX90393 magnetometer.
    :param i2c_bus: The `busio.I2C` object to use. This is the only
    required parameter.
    :param int address: (optional) The I2C address of the device.
    :param int gain: (optional) The gain level to apply.
    :param bool debug: (optional) Enable debug output.
    """

    def __init__(self, i2c_bus, address=0x0C, gain=GAIN_1X, debug=False):
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._debug = debug
        self._status_last = 0
        self._res_current = _RES_2_15
        self._gain_current = gain

        # Put the device in a known state to start
        self.reset()

        # Set gain to the supplied level
        self.gain = self._gain_current

    def _transceive(self, payload, rxlen=0):
        """
        Writes the specified 'payload' to the sensor
        Returns the results of the write attempt.
        :param bytearray payload: The byte array to write to the sensor
        :param rxlen: (optional) The numbers of bytes to read back (default=0)
        """
        # Read the response (+1 to account for the mandatory status byte!)
        data = bytearray(rxlen + 1)

        if len(payload) == 1:
            # Transceive with repeated start
            with self.i2c_device as i2c:
                i2c.write_then_readinto(payload, data)
        else:
            # Write 'value' to the specified register
            # TODO: Check this. It's weird that the write is accepted but the read is naked.
            with self.i2c_device as i2c:
                i2c.write(payload, stop=False)

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
        self._status_last = data[0]
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

    @property
    def gain(self):
        """
        Gets the current gain setting for the device.
        """
        return self._gain_current

    @gain.setter
    def gain(self, value):
        """
        Sets the gain for the device.
        """
        if value > GAIN_1X or value < GAIN_5X:
            raise ValueError("Invalid GAIN setting")
        if self._debug:
            print("\tSetting gain: {}".format(value))
        self._gain_current = value
        self._transceive(
            bytes(
                [
                    _CMD_WR,
                    0x00,
                    self._gain_current << _GAIN_SHIFT | _HALLCONF,
                    (_CMD_REG_CONF1 & 0x3F) << 2,
                ]
            )
        )

    def display_status(self):
        """
        Prints out the content of the last status byte in a human-readble
        format.
        """
        avail = 0
        if self._status_last & 0b11 > 0:
            avail = 2 * (self._status_last & 0b11) + 2
        print("STATUS register = 0x{0:02X}".format(self._status_last))
        print("BURST Mode               :", (self._status_last & (1 << 7)) > 0)
        print("WOC Mode                 :", (self._status_last & (1 << 6)) > 0)
        print("SM Mode                  :", (self._status_last & (1 << 5)) > 0)
        print("Error                    :", (self._status_last & (1 << 4)) > 0)
        print("Single error detection   :", (self._status_last & (1 << 3)) > 0)
        print("Reset status             :", (self._status_last & (1 << 2)) > 0)
        print("Response bytes available :", avail)

    def read_reg(self, reg):
        """
        Gets the current value of the specified register.
        """
        # Write 'value' to the specified register
        payload = bytes([_CMD_RR, reg << 2])
        with self.i2c_device as i2c:
            i2c.write(payload)

        # Read the response (+1 to account for the mandatory status byte!)
        data = bytearray(3)
        with self.i2c_device as i2c:
            i2c.readinto(data)
        # Unpack data (status byte, big-endian 16-bit register value)
        self._status_last, val = struct.unpack(">Bh", data)
        if self._debug:
            print("\t[{}]".format(time.monotonic()))
            print("\t Writing :", [hex(b) for b in payload])
            print("\tResponse :", [hex(b) for b in data])
            print("\t  Status :", hex(data[0]))
        return val

    def reset(self):
        """
        Performs a software reset of the sensor.
        """
        if self._debug:
            print("Resetting sensor")
        time.sleep(2)
        self._status_last = self._transceive(bytes([_CMD_RT]))
        # burn a read post reset
        try:
            self.magnetic
        except OSError:
            pass
        return self._status_last

    @property
    def read_data(self):
        """
        Reads a single X/Y/Z sample from the magnetometer.
        """
        delay = 0.01

        # Set the device to single measurement mode
        self._transceive(bytes([_CMD_SM | _CMD_AXIS_ALL]))

        # Insert a delay since we aren't using INTs for DRDY
        time.sleep(delay)

        # Read the 'XYZ' data as three signed 16-bit integers
        data = self._transceive(bytes([_CMD_RM | _CMD_AXIS_ALL]), 6)
        self._status_last, m_x, m_y, m_z = struct.unpack(">Bhhh", data)

        # Return the raw int values if requested
        return (m_x, m_y, m_z)

    @property
    def magnetic(self):
        """
        The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats.
        """
        x, y, z = self.read_data

        # Convert the raw integer values to uT based on gain and resolution
        x *= _LSB_LOOKUP[self._gain_current][self._res_current][0]
        y *= _LSB_LOOKUP[self._gain_current][self._res_current][0]
        z *= _LSB_LOOKUP[self._gain_current][self._res_current][1]

        return x, y, z
