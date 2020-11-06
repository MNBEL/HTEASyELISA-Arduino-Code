import csv
import matplotlib.pyplot as plt
import numpy as np
import copy

class Sweep_Data:
    
    def __init__(self, filename):
        self.filename = filename
        self.frequencies = []
        self.impedances = []
        self.phases = []
    
    def read_ZM(self):
        with open(self.filename, newline='') as csvfile:
            content = csv.reader(csvfile, delimiter=',')
            header_len = 32
            for i, row in enumerate(content):
                if i <= header_len:
                    pass
                else:
                    self.frequencies.append(row[2])
                    self.impedances.append(row[4])
                    self.phases.append(row[5])
        self.frequencies = np.asarray(self.frequencies).astype(float)
        self.impedances = np.asarray(self.impedances).astype(float)
                    
    def read_PMOD(self):
        with open(filename, newline='') as csvfile:
            content = csv.reader(csvfile, delimiter=',')
            for row in content:
                self.frequencies.append(row[0])
                self.impedances.append(row[1])
        self.frequencies = np.asarray(self.frequencies).astype(float)
        self.impedances = np.asarray(self.impedances).astype(float)
        
    def model_splpf(self, frequencies, R, C):
        d1 = np.power(1/R, 2)
        d2 = np.power(frequencies * 2 * np.pi * C, 2)
        zs = 1/np.sqrt(d1+d2)
        self.frequencies = frequencies
        self.impedances = zs
        
    def model_dplpf(self, frequencies, R1, R2, C1, C2):
        d1 = np.power(1/R1, 2)
        d2 = np.power(frequencies * 2 * np.pi * C1, 2)
        d3 = np.power(1/R2, 2)
        d4 = np.power(frequencies * 2 * np.pi * C2, 2)
        zs = 1/np.sqrt(d1+d2) + 1/np.sqrt(d3+d4)
        self.frequencies = frequencies
        self.impedances = zs
        
    def r_c_p_correction(self, C):
        d1 = np.power(1/self.impedances, 2)
        d2 = np.power(self.frequencies * 2 * np.pi * C, 2)
        zes = 1/np.sqrt(d1-d2)
        self.impedances = zes
        
# should this all be done with dictionaries?
class Interpolator:
    def __init__(self, fcs, zcm_fc_data, zct_fc_data):
        self.fcs = fcs
        self.zcm_fc_data = zcm_fc_data
        self.zct_fc_data = zct_fc_data
        
        self.zm = 0
        self.fm = 0
        
        self.fc_minus = 0
        self.fc_plus = 0
        self.fc_minus_i = 0
        self.fc_plus_i = 0
        
        self.zcm_minus_fc_minus = 0
        self.zct_minus_fc_minus = 0
        self.zcm_plus_fc_minus = 0
        self.zct_plus_fc_minus = 0
        self.zcm_minus_fc_plus = 0
        self.zct_minus_fc_plus = 0
        self.zcm_plus_fc_plus = 0
        self.zct_plus_fc_plus = 0
        
        self.ze_fc_minus = 0
        self.ze_fc_plus = 0
        
        self.ze = 0
        
    def adj_freqs(self):
        i = np.searchsorted(self.fcs, self.fm)
        self.fc_minus = self.fcs[i-1]
        self.fc_plus = self.fcs[i]
        self.fc_minus_i = i-1
        self.fc_plus_i = i
        
    def adj_zcs(self):
        self.adj_freqs()
        
        zcms_fc_minus = self.zcm_fc_data[:,self.fc_minus_i]
        i = np.searchsorted(zcms_fc_minus, self.zm)
        self.zcm_minus_fc_minus = zcms_fc_minus[i-1]
        self.zcm_plus_fc_minus = zcms_fc_minus[i]
        
        zcms_fc_plus = self.zcm_fc_data[:,self.fc_plus_i]
        i = np.searchsorted(zcms_fc_plus, self.zm)
        self.zcm_minus_fc_plus = zcms_fc_plus[i-1]
        self.zcm_plus_fc_plus = zcms_fc_plus[i]
        
        zcts_fc_minus = self.zct_fc_data[:,self.fc_minus_i]
        i = np.searchsorted(zcts_fc_minus, self.zm)
        self.zct_minus_fc_minus = zcts_fc_minus[i-1]
        self.zct_plus_fc_minus = zcts_fc_minus[i]
        
        zcts_fc_plus = self.zct_fc_data[:,self.fc_plus_i]
        i = np.searchsorted(zcts_fc_plus, self.zm)
        self.zct_minus_fc_plus = zcts_fc_plus[i-1]
        self.zct_plus_fc_plus = zcts_fc_plus[i]
        
    def adj_zes(self):
        self.adj_zcs()
        
        slope_minus = (self.zct_plus_fc_minus - self.zct_minus_fc_minus) \
                    / (self.zcm_plus_fc_minus - self.zcm_minus_fc_minus)
        self.ze_fc_minus = slope_minus * (self.zm - self.zcm_minus_fc_minus) \
                    + self.zct_minus_fc_minus
                    
        slope_plus = (self.zct_plus_fc_plus - self.zct_minus_fc_plus) \
                    / (self.zcm_plus_fc_plus - self.zcm_minus_fc_plus)
        self.ze_fc_plus = slope_plus * (self.zm - self.zcm_minus_fc_plus) \
                    + self.zct_minus_fc_plus

    def interpolate(self, zm, fm):
        self.zm = zm
        self.fm = fm
        self.adj_zes()
        
        slope = (self.ze_fc_plus - self.ze_fc_minus) \
                / (self.fc_plus - self.fc_minus)
                
        ze = slope * (self.fm - self.fc_minus) + self.ze_fc_minus
        
        self.ze = ze
        return ze
    
    def plot(self, ax):
        ax.plot([self.zcm_minus_fc_plus, \
                self.zcm_plus_fc_plus], \
                [self.zct_minus_fc_plus, \
                self.zct_plus_fc_plus], 'yo')
            
        ax.plot([self.zcm_minus_fc_minus, \
                self.zcm_plus_fc_minus], \
                [self.zct_minus_fc_minus, \
                self.zct_plus_fc_minus], 'go')
            
        ax.plot([self.zm, self.zm], \
                [self.ze_fc_minus, self.ze_fc_plus], 'bo')
            
        ax.plot(self.zm, self.ze, 'ro')
        ax.plot([0,self.zm], [self.ze, self.ze], 'r--')

filenames_ZM = [
                '100.csv',\
                '200.csv',\
                '1k.csv',\
                '2k.csv',\
                '10k.csv',\
                '20k.csv',\
                '100k.csv',\
                '200k.csv',\
                # '17.csv',\
                ]
    
filenames_PMOD = [
                # 'standard.csv',\
                ]
models_splpf = {
                # 'R||C = 1.1e-11' : [100000, 1.05e-11],\
                # 'R||C = 2.5e-11' : [100000, 6.6e-11],\
                # 'R||C = 2.5e-11_test' : [90000, 2.5e-11],\
                }
correction_names = {
                # '100K_corrected' : ['100k.csv', 6.6e-11],\
                # '200K_corrected' : ['200k.csv', 6.6e-11],\
                # 'test_corrected_C=2.5e-11' : ['R||C = 2.5e-11_test', 2.5e-11],\
                    }

sweeps = {}
for i, filename in enumerate(filenames_ZM):
    sweeps[filename] = Sweep_Data(filename)
    sweeps[filename].read_ZM()
for i, filename in enumerate(filenames_PMOD):
    sweeps[filename] = Sweep_Data(filename)
    sweeps[filename].read_PMOD()
for i, model_name in enumerate(models_splpf):
    model = models_splpf[model_name]
    sweeps[model_name] = Sweep_Data(model_name)
    sweeps[model_name].model_splpf(np.arange(1000, 95000, 1000), model[0], model[1])
for i, correction_name in enumerate(correction_names):
    correction = correction_names[correction_name]
    sweeps[correction_name] = copy.deepcopy(sweeps[correction[0]])
    sweeps[correction_name].r_c_p_correction(correction[1])

zcm_fc_data = np.zeros((0))

fig, ax = plt.subplots(figsize=(20,10))
figlog, axlog = plt.subplots(figsize=(20,10))
for filename in sweeps:
    sweep = sweeps[filename]
    freqs = sweep.frequencies
    zs = sweep.impedances
    zcm_fc_data = np.concatenate((zcm_fc_data, zs), axis=0)
    ax.plot(freqs, zs, label=filename)
    axlog.plot(np.log10(freqs), np.log10(zs), label=filename)
ax.legend()
axlog.legend()

fcs = freqs
zct_fc_data = [100, 200, 1e3, 2e3, 10e3, 20e3, 100e3, 200e3]
zct_fc_data = np.resize(zct_fc_data, (len(sweeps)* len(zs)))
zct_fc_data = np.resize(zct_fc_data, (len(zs), len(sweeps)))
zct_fc_data = np.transpose(zct_fc_data)
zcm_fc_data = np.resize(zcm_fc_data, (len(sweeps), len(zs)))
# # visual for interpolation
figc, axc = plt.subplots(figsize=(20,10))
axc.plot(zcm_fc_data, zct_fc_data[:,0])
# axc.plot(zct_fc_data, zct_fc_data[:,0])

# need to check all edge cases and fix insert!!!!!!!!
# and decide how to handle edge cases.
interpolator = Interpolator(fcs, zcm_fc_data, zct_fc_data)
ze = interpolator.interpolate(110e3, 10000)
ax.plot(10e3, 110e3, 'bo')
ax.plot(10e3, ze, 'ro')
ax.plot([0,10e3], [ze, ze], 'r--')
interpolator.plot(axc)
axc.set_xlabel('Measured')
axc.set_ylabel('True')
