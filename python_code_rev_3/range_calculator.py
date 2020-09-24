# -*- coding: utf-8 -*-
"""
Created on Wed Aug  5 13:56:14 2020

@author: josia
"""
# estimates of minimum and maximum impedance measurements

PGA = [1, 5]
RFB = [20, 100000]
Vpp = [1.8, 0.88, 0.35, 0.18]
Vmin = 0.00073

for vpp in Vpp:
    for rfb in RFB:
        for pga in PGA:
            Zmin = rfb * pga / (1.5 + vpp / 2)
            Zmax = rfb * pga / Vmin
            print(pga, rfb, vpp, Zmin, Zmax)