#!/usr/bin/env python3
"""
Sanity-check + clear the MAX7219 dot matrix display.

Lights every pixel for a moment (verifies all LEDs and the SPI chain are
alive), then clears the display.
"""

import argparse
import time

from luma.core.interface.serial import noop, spi
from luma.core.render import canvas
from luma.led_matrix.device import max7219


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spi-port', type=int, default=0)
    parser.add_argument('--spi-device', type=int, default=0)
    parser.add_argument('--cascaded', type=int, default=4,
                        help='Number of cascaded MAX7219 modules (default 4)')
    parser.add_argument('--hold-seconds', type=float, default=1.0)
    args = parser.parse_args()

    serial = spi(port=args.spi_port, device=args.spi_device, gpio=noop())
    device = max7219(serial, cascaded=args.cascaded)

    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline='white', fill='white')
    time.sleep(args.hold_seconds)

    device.clear()


if __name__ == '__main__':
    main()
