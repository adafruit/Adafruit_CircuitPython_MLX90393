import sys
import busio
import board
import time

from adafruit_mlx90393 import MLX90393

i2c = busio.I2C(board.SCL, board.SDA)
sensor = MLX90393(i2c)

res = sensor.reset()
sensor.display_status(res[0])
