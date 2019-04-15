#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function
import time
import os
import random
import serial
from serial.tools import list_ports
from .labtime import labtime
from .version import __version__

sep = ' '   # command/value separator in TCLab firmware

arduinos = [('USB VID:PID=16D0:0613', 'Arduino Uno'),
            ('USB VID:PID=1A86:7523', 'NHduino'),
            ('USB VID:PID=2341:8036', 'Arduino Leonardo'),
            ('USB VID:PID=2A03', 'Arduino.org device'),
            ('USB VID:PID', 'unknown device'),
            ]

_sketchurl = 'https://github.com/jckantor/TCLab-sketch'
_connected = False


def clip(val, lower=0, upper=100):
    """Limit value to be between lower and upper limits"""
    return max(lower, min(val, upper))


def command(name, argument, lower=0, upper=100):
    """Construct command to TCLab-sketch."""
    return name + sep + str(clip(argument, lower, upper))


def find_arduino(port=''):
    """Locates Arduino and returns port and device."""
    comports = [tuple for tuple in list_ports.comports() if port in tuple[0]]
    for port, desc, hwid in comports:
        for identifier, arduino in arduinos:
            if hwid.startswith(identifier):
                return port, arduino
    print('--- Serial Ports ---')
    for port, desc, hwid in list_ports.comports():
        print(port, desc, hwid)
    return None, None


class AlreadyConnectedError(BaseException):
    pass


class TCLab(object):
    def __init__(self, port='', debug=False):
        self.debug = debug
        print("TCLab version", __version__)
        self.port, self.arduino = find_arduino(port)
        if self.port is None:
            raise RuntimeError('No Arduino device found.')

        try:
            self.connect(baud=9600)
        except AlreadyConnectedError:
            raise
        except Exception as e:
            try:
                print (str(e))
                self.sp.close()
                self.connect(baud=115200)
                print('Could not connect at high speed, but succeeded at low speed.')
                print('This may be due to an old TCLab firmware.')
                print('New Arduino TCLab firmware available at:')
                print(_sketchurl)
            except:
                raise RuntimeError('Failed to Connect.')

        self.sp.readline().decode('UTF-8')
        self.version = self.send_and_receive('VER')
        if self.sp.isOpen():
            print(self.arduino, 'connected on port', self.port,
                  'at', self.baud, 'baud.')
            print(self.version + '.')
        labtime.set_rate(1)
        labtime.start()
        self._P1 = 200.0
        self._P2 = 100.0
        self.Q2(0)
        self.sources = [('T1', self.scan),
                        ('T2', None),
                        ('Q1', None),
                        ('Q2', None),
                        ]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return

    def connect(self, baud):
        """Establish a connection to the arduino

        baud: baud rate"""
        global _connected

        if _connected:
            raise AlreadyConnectedError('You already have an open connection')

        _connected = True
        self.sp = serial.Serial(port=self.port, baudrate=baud, timeout=2)
        time.sleep(2)
        self.Q1(0)  # fails if not connected
        self.baud = baud

    def close(self):
        """Shut down TCLab device and close serial connection."""
        global _connected

        self.Q1(0)
        self.Q2(0)
        self.send_and_receive('X')
        self.sp.close()
        _connected = False
        print('TCLab disconnected successfully.')
        return

    def send(self, msg):
        """Send a string message to the TCLab firmware."""
        self.sp.write((msg + '\r\n').encode())
        if self.debug:
            print('Sent: "' + msg + '"')
        self.sp.flush()

    def receive(self):
        """Return a string message received from the TCLab firmware."""
        msg = self.sp.readline().decode('UTF-8').replace('\r\n', '')
        if self.debug:
            print('Return: "' + msg + '"')
        return msg

    def send_and_receive(self, msg, convert=str):
        """Send a string message and return the response"""
        self.send(msg)
        return convert(self.receive())

    def LED(self, val=100):
        """Flash TCLab LED at a specified brightness for 10 seconds."""
        return self.send_and_receive(command('LED', val), float)

    @property
    def T1(self):
        """Return a float denoting TCLab temperature T1 in degrees C."""
        temp = self.send_and_receive('T1', float)
        return temp

    @property
    def T2(self):
        """Return a float denoting TCLab temperature T2 in degrees C."""
        return self.send_and_receive('T2', float)

    @property
    def P1(self):
        """Return a float denoting maximum power of heater 1 in pwm."""
        return self._P1

    @P1.setter
    def P1(self, val):
        """Set maximum power of heater 1 in pwm, range 0 to 255."""
        self._P1 = self.send_and_receive(command('P1', val, 0, 255), float)

    @property
    def P2(self):
        """Return a float denoting maximum power of heater 2 in pwm."""
        return self._P2

    @P2.setter
    def P2(self, val):
        """Set maximum power of heater 2 in pwm, range 0 to 255."""
        self._P2 = self.send_and_receive(command('P2', val, 0, 255), float)

    def Q1(self, val=None):
        """Get or set TCLab heater power Q1

        val: Value of heater power, range is limited to 0-100

        return clipped value."""
        if val is None:
            msg = 'R1'
        else:
            msg = 'Q1' + sep + str(clip(val))
        return self.send_and_receive(msg, float)

    def Q2(self, val=None):
        """Get or set TCLab heater power Q2

        val: Value of heater power, range is limited to 0-100

        return clipped value."""
        if val is None:
            msg = 'R2'
        else:
            msg = 'Q2' + sep + str(clip(val))
        return self.send_and_receive(msg, float)

    def scan(self):
        #self.send('SCAN')
        T1 = self.T1  # float(self.receive())
        T2 = self.T2  # float(self.receive())
        Q1 = self.Q1()  # float(self.receive())
        Q2 = self.Q2()  # float(self.receive())
        return T1, T2, Q1, Q2

    # Define properties for Q1 and Q2
    U1 = property(fget=Q1, fset=Q1, doc="Heater 1 value")
    U2 = property(fget=Q2, fset=Q2, doc="Heater 2 value")
