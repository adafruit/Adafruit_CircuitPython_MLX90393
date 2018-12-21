Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-mlx90393/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/mlx90393/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.com/adafruit/Adafruit_CircuitPython_MLX90393.svg?branch=master
    :target: https://travis-ci.com/adafruit/Adafruit_CircuitPython_MLX90393
    :alt: Build Status

Adafruit CircuitPython driver for the MLX90393 3-axis magnetometer.

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Installing from PyPI
--------------------

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-mlx90939/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-mlx90939

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-mlx90939

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-mlx90939

Usage Example
=============

.. code-block:: python3

    import time
    import busio
    import board

    import adafruit_mlx90393

    I2C_BUS = busio.I2C(board.SCL, board.SDA)
    SENSOR = adafruit_mlx90393.MLX90393(I2C_BUS, gain=adafruit_mlx90393.GAIN_1X)

    while True:
        MX, MY, MZ = SENSOR.read_data(delay=1.0, raw=False)
        print("[{}]".format(time.monotonic()))
        print("X: {} uT".format(MX))
        print("Y: {} uT".format(MY))
        print("Z: {} uT".format(MZ))
        # Display the status field if an error occured, etc.
        if SENSOR.last_status > adafruit_mlx90393.STATUS_OK:
            SENSOR.display_status()

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_MLX90393/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

Zip release files
-----------------

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit_circuitpython_mlx90393 --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
