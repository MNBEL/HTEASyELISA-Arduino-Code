# -*- coding: utf-8 -*-
"""
Created on Tue Aug  4 16:09:03 2020

@author: josia
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

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

ser = serial.Serial('COM4', 38400)

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
recieved = 0
while not recieved:
    while not ser.in_waiting:
        ser.write(b'a')
        time.sleep(1)
    try:
        b = ser.readline()
        recieved = 1
        print(b)
    except:
        ser.close()
        ser = serial.Serial('COM4', 38400)
ser.close()

b = b.decode()
b = b.replace('\r\n', '') # remove endline
if b[0] == '[' and b[-1] == ']':
    print('seems good')
b = b[1:-2]
b = b.split(',')

b_array = np.array(b, dtype='float')
b_array = np.reshape(b_array, (int(len(b_array)/4), 4))
frame = pd.DataFrame(data=b_array, columns=['Well', 'Freq', 'Imp', 'Phi'])
frame = frame.astype({'Well': 'int'})

impedances = np.reshape(np.array(frame['Imp']), (12,8)) # not sure why this is akward
nums = np.reshape(np.array(frame['Well']), (12,8))

fig, ax = plt.subplots()
sns.heatmap(impedances, linewidths=1, linecolor='w', square=True, ax=ax, cbar_kws={'label': 'Impedance (Z)'})
sns.heatmap(nums, square=True, ax=ax, annot=True, alpha=0, annot_kws={'color':'w'}, cbar=False, xticklabels=False, yticklabels=False)
fig.tight_layout()
plt.show()

fig.savefig('C:/Users/josia/Desktop/fig', bbox_inches='tight')

# np.savetxt('impedance_data.csv', impedances_mapped, delimiter=',')
# np.savetxt('phase_data', phases_mapped, delimiter=',')