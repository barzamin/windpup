#!/usr/bin/env python3
import serial
import click
import os
import time

@click.command()
@click.argument('pup_port')
@click.argument('modem_port')
def test(pup_port, modem_port):
    # os.system(f'idf.py -p {pup_port} flash')
    input('reset board and hit enter... ')
    with serial.Serial(modem_port, 9600, timeout=1) as ser:
        ser.reset_input_buffer()

        ser.write(b'woof woof bark bark\n')
        assert ser.readline() == b'woof woof bark bark\n'
        click.secho('rx, tx passed!', fg='green')

        assert ser.cts == False
        ser.write(b'\n')
        time.sleep(0.1)
        assert ser.cts == True
        click.secho('RTS pass!', fg='green')

        assert ser.dsr == False
        ser.write(b'\n')
        time.sleep(0.1)
        assert ser.dsr == True
        click.secho('DTR pass!', fg='green')

        assert ser.ri == False
        ser.write(b'\n')
        time.sleep(0.1)
        assert ser.ri == True
        click.secho('RI pass!', fg='green')


if __name__ == '__main__':
    test()
