# -*- coding: utf-8 -*-
"""
Created on Tue Aug  4 16:09:03 2020

@author: josia
"""

import serial
import csv
# import time
import numpy as np
import matplotlib.pyplot as plt

ser = serial.Serial('COM4', 38400)

data_points = {}
length_check = 1
while (length_check):
    b = ser.readline()
    # this should do an error check to start at the right place
    string = b.decode()
    fields = string.split()
    freq = float(fields[0])
    imp = float(fields[1])
    phase = float(fields[2])
    if not (freq in data_points):
        data_points[freq] = [imp, phase]
    else:
        length_check = 0
print(len(data_points))
ser.close()

impedance = 100000
phase = 0
with open(str(impedance)+'_'+str(phase)+'.csv', mode='w',newline='') as output:
    output_writer = csv.writer(output, delimiter=',')
    
    for freq in data_points:
        output_writer.writerow([freq, data_points[freq][0], data_points[freq][1]])

    output.close()


