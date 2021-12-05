'''
Author: Chengkun Li
LastEditors: Chengkun Li
Date: 2021-12-05 17:26:43
LastEditTime: 2021-12-05 18:18:46
Description: Modify here please
FilePath: /Mobile_Soccer/trial.py
'''
from Thymio import Thymio
import os
import sys

import glob
import serial


def available_serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


if __name__ == '__main__':
    import time
    
    with Thymio.serial(port="COM5", refreshing_rate=0.1) as th:
        while True:
            try:
                th.set_var("motor.left.target", 0)
                th.set_var("motor.right.target", 0)
            except KeyError:
                pass
            time.sleep(0.2)