# -*- coding: utf-8 -*-
"""
Created on Tue Aug  4 16:09:03 2020

@author: josia
"""

import serial
# import time
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
               ' frequency: '+str(self.frequencies)+ \
               ' impedance: '+str(self.impedances[0])+ \
               ' phase: '+str(self.phase[0])
               
    def add_point(self, frequency, impedance, phase):
        self.frequencies.append(frequency)
        self.impedances.append(impedance)
        self.phases.append(phase)

ser = serial.Serial('COM4', 38400)

wells = []
for i in range(1, 97, 1):
    well = Well(i)

impedances_mapped = np.zeros((12,8))
phases_mapped = np.zeros((12,8))

beginning = 0
done = 0

# this is a junk header ???
b = ser.readline()
print(b)
# wait till first well reading then read till 96. This is fine as long as 
# nothing is skipped
while (done == 0):
    b = ser.readline()
    string = b.decode()
    fields = string.split()
    well = Well(int(fields[0]), float(fields[1]), float(fields[2]))
    if (well.number == 1):
        beginning = 1
        print("beginning")
    if (beginning == 1):
        print(well)
        wells.append(well)
        if (well.number == 96):
            done = 1
            print("end")
            
# must close to run consecutively... unless if the start serial 
# was error checked...
#... opening serial resets arduino... so really just need to add a check
# see if serial port is already open!
# print("closing serial")
ser.close()

# used to convert relevant well attributes into array form for plotting
impedances = []
phases = []
# create 12 by 8 array
for well in wells:
    impedances.append(well.impedance)
    phases.append(well.phase)
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










