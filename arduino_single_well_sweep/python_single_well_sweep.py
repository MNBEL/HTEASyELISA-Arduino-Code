# -*- coding: utf-8 -*-
"""
Created on Tue Aug  4 16:09:03 2020

@author: josia
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# Well object to create well list
class Well:
    number = 0
    impedance = 0
    phase = 0
    
    def __init__(self, number):
        self.number = number
        self.frequencies = [];
        self.impedances = [];
        self.phases = []
        
    def __str__(self):
        return 'Well (number: '+str(self.number)+ \
               ' frequency: '+str(self.frequencies[0])+ \
               ' impedance: '+str(self.impedances[0])+ \
               ' phase: '+str(self.phases[0])
               
    def add_point(self, frequency, impedance, phase):
        self.frequencies.append(frequency)
        self.impedances.append(impedance)
        self.phases.append(phase)

ser = serial.Serial('COM5', 38400)

well_number = 7
well = Well(well_number)

num = 0
freq = 0
imp = 0
phase = 0
length_check = 1
while (length_check):
    while not ser.in_waiting:
        ser.write(b'0')
        time.sleep(1)
    try:
        b = ser.readline()
        string = b.decode()
        fields = string.split()
        num = int(fields[0])
        freq = float(fields[1])
        imp = float(fields[2])
        phase = float(fields[3])
        if not (freq in well.frequencies):
            well.add_point(freq, imp, phase)
        else:
            length_check = 0
    except:
        ser.close()
        ser = serial.Serial('COM5', 38400)
ser.close()

# used to convert relevant well attributes into array form for plotting
impedances = []
phases = []

fig, ax = plt.subplots(figsize=(20,10))
ax.plot(well.frequencies, well.impedances)