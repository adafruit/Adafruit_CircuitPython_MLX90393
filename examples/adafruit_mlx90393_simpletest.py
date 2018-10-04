import busio
import board
import time

from adafruit_mlx90393 import MLX90393

i2c = busio.I2C(board.SCL, board.SDA)
sensor = MLX90393(i2c, debug=False)

while True:
    status, x, y, z = sensor.read_data()
    if not status & 0x04:
        print("X: {} uT".format(x))
        print("Y: {} uT".format(y))
        print("Z: {} uT".format(z))
    else:
        # Something went wrong, error bit set:
        sensor.display_status(status)

    time.sleep(1)
