import busio
import board
import time

from adafruit_mlx90393 import MLX90393

i2c = busio.I2C(board.SCL, board.SDA)
sensor = MLX90393(i2c, debug=False)

while True:
    x, y, z = sensor.read_data(delay=1.0, raw=False)
    print("[{}]".format(time.monotonic()))
    print("X: {} uT".format(x))
    print("Y: {} uT".format(y))
    print("Z: {} uT".format(z))
    # Display the status field if something odd happened
    if (sensor.last_status > 0):
        print("S: {}".format(hex(sensor.last_status)))
