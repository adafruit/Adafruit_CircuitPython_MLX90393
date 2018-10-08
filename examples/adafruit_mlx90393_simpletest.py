"""
Minimal example of getting data from the MLX90393 magnetometer.
"""
import time
import busio
import board

from adafruit_mlx90393 import MLX90393

I2C_BUS = busio.I2C(board.SCL, board.SDA)
SENSOR = MLX90393(I2C_BUS, debug=False)

while True:
    MX, MY, MZ = SENSOR.read_data(delay=1.0, raw=False)
    print("[{}]".format(time.monotonic()))
    print("X: {} uT".format(MX))
    print("Y: {} uT".format(MY))
    print("Z: {} uT".format(MZ))
    # Display the status field if > 3
    if SENSOR.last_status > 3:
        SENSOR.display_status()
