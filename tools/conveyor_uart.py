"""Relay a PC session's stdin and stdout to the UNO Q MCU UART on Linux."""

import argparse
import fcntl
import os
import select
import sys
import termios
import tty


def relay(uart):
    fcntl.flock(uart, fcntl.LOCK_EX | fcntl.LOCK_NB)
    saved = termios.tcgetattr(uart)
    try:
        tty.setraw(uart)
        settings = termios.tcgetattr(uart)
        settings[2] &= ~termios.CRTSCTS
        settings[4] = settings[5] = termios.B115200
        termios.tcsetattr(uart, termios.TCSAFLUSH, settings)
        while True:
            ready, _, _ = select.select([0, uart], [], [])
            for source in ready:
                data = os.read(source, 4096)
                if not data:
                    return
                dest = uart if source == 0 else 1
                while data:
                    data = data[os.write(dest, data):]
    finally:
        termios.tcsetattr(uart, termios.TCSANOW, saved)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/ttyHS1')
    args = parser.parse_args()
    try:
        with open(args.device, 'r+b', buffering=0) as port:
            relay(port.fileno())
    except OSError as error:
        sys.exit(str(error))
