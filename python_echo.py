# -*- coding: utf-8 -*-
"""
Created on Tue Aug  4 16:09:03 2020

@author: josia
"""

import serial

ser = serial.Serial('COM4', 38400)

while(1):
    ser.write(b'c')
    print(ser.readline())