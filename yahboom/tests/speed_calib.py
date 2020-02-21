#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 17:11:14 2020

@author: chrisolen
"""

import matplotlib.pyplot as plt
import numpy as np

from pylab import *
from scipy.optimize import curve_fit

xx = np.arange(1,70,1)
# secondsx
x = np.array([5,5,5,10,10,10,15,15,15,20,20,20,30,30,30,40,40,40,50,50,50,60,60,60,70,70,70])
# cm
y = np.array([7.21,6.32,7.28,3.74,4.346,3.93,2.645,3.42,3.66,2.61,2.72,2.64,1.87,1.82,2.00,1.64,1.41,1.55,1.23,1.37,1.26,1.12,1.20,1.20,1.14,1.13,1.12])
# cm/s
y = 75 / y

def func(x, a, b, c, d):
    return a*np.exp(-c*(x-b))+d

popt, pcov = curve_fit(func, x, y, p0=(10, 10, 10, 10))

yy = func(xx, *popt)

plt.plot(x,y,'ko')
plt.plot(x,y)
plt.plot(xx, yy)

a, b, c, d = popt


