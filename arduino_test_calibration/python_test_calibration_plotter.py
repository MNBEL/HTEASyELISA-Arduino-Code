
filenames = ['100000_0.csv', \
             '100001_0.csv', \
         ]
    
import csv
import numpy as np
import matplotlib.pyplot as plt
    

all_data = {}
for filename in filenames:
    freqs = []
    impedances = []
    phases = []
    with open(filename, mode='r') as file:
        file_reader = csv.reader(file, delimiter=',')
        for row in file_reader:
            freqs.append(row[0])
            impedances.append(row[1])
            phases.append(row[2])
    file.close()
    
    all_data[filename] = [freqs, impedances, phases]

freqs = np.zeros((len(filenames), len(all_data[filenames[0]][0])))
impedances = np.zeros((len(filenames), len(all_data[filenames[0]][0])))
phases = np.zeros((len(filenames), len(all_data[filenames[0]][0])))

for i in range(len(filenames)):
    freqs[i][:] = all_data[filenames[i]][0]
    impedances[i][:] = all_data[filenames[i]][1]
    phases[i][:] = all_data[filenames[i]][2]

fig, ax1 = plt.subplots()

ax1.set_xlabel('freq (Hz)')
ax1.set_ylabel('impedance (Z)')
# ax1.plot(freqs[0], np.transpose(impedances))
ax2 = ax1.twinx()
ax2.set_ylabel('phase (radian)')
# ax2.plot(freqs[0], np.transpose(phases))

colors = ['r', 'b', 'y', 'c']
# for i in range(len(filenames)):
for i in [0]:
    ax1.plot(freqs[0], impedances[i], colors[i])
    ax2.plot(freqs[0], phases[i], colors[i])
    imp_cal = filenames[i].split('_')[0]
    ax1.plot(freqs[0], np.zeros(len(all_data[filenames[0]][0]))+int(imp_cal), colors[i+2])
    phase_cal = filenames[i].split('_')[1].split('.')[0]
    ax2.plot(freqs[0], np.zeros(len(all_data[filenames[0]][0]))+int(phase_cal), colors[i+2])