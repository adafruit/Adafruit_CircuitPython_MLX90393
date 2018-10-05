import busio
import board
import time

from adafruit_mlx90393 import MLX90393

i2c = busio.I2C(board.SCL, board.SDA)
sensor = MLX90393(i2c, debug=False)

while True:
    x, y, z = sensor.read_data(delay=1.0, raw=True)
    print("[{}]".format(time.monotonic()))
    print("X: {} lsb".format(x))
    print("Y: {} lsb".format(y))
    print("Z: {} lsb".format(z))
    # Display the status field if something odd happened
    if (sensor.last_status() > 0):
        print("S: {}".format(hex(sensor.last_status())))
