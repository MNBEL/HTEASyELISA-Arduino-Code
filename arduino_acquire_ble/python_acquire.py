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

ser = serial.Serial('COM3', 38400)

wells = []
for i in range(1, 97, 1):
    wells.append(Well(i))

impedances_mapped = np.zeros((12,8))
phases_mapped = np.zeros((12,8))

num = 0
freq = 0
imp = 0
phase = 0
nums = {}
length_check = 1
while (length_check):
    while not ser.in_waiting:
        ser.write(b'0')
        time.sleep(1)
    try:
        b = ser.readline()
        string = b.decode()
        fields = string.split(",")
        num = int(fields[0])
        freq = float(fields[1])
        imp = float(fields[2])
        phase = float(fields[3])
        # check if well has already been read, if so then assume all wells 
        # have been read
        if not (num in nums):
            wells[num - 1].add_point(freq, imp, phase)
            # nums[num] = 0 ???
            nums[num - 1] = num
        else:
            length_check = 0
    except:
        ser.close()
        ser = serial.Serial('COM3', 38400)
ser.close()

# used to convert relevant well attributes into array form for plotting
impedances = []
phases = []
# create 12 by 8 array
for well in wells:
    impedances.append(well.impedances[0])
    phases.append(well.phases[0])
impedances_mapped = np.array(impedances)
impedances_mapped = np.resize(impedances_mapped, (12,8))
phases_mapped = np.array(phases)
phases_mapped = np.resize(phases_mapped, (12, 8))
print(impedances_mapped)
print(phases_mapped)

# plots as heatmap... could use some better formatting with
# DC of colorbar at 0... also easier to read numbering
fig, ax = plt.subplots(figsize=(20,10))
ax.set_xticks(np.arange(8))
ax.set_yticks(np.arange(12))
im = ax.imshow(impedances_mapped)
fig.colorbar(im)
for i in range(12):
    for j in range(8):
        text = "{:2.2e}".format(impedances_mapped[i, j])
        ax.text(j, i, text, color='w', fontsize=8, ha="center", va="center")

np.savetxt('impedance_data.csv', impedances_mapped, delimiter=',')
np.savetxt('phase_data', phases_mapped, delimiter=',')