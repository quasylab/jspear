# -*- coding: utf-8 -*-
"""
Created on Thu May  9 11:51:13 2024

@author: 20235409
"""

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean
import pandas as pd


df_n = pd.read_csv('xy_nominal.csv', names=['x_n', 'y_n'])
df_f = pd.read_csv('xy_feedback.csv', names=['x_f', 'y_f'])

fix, ax = plt.subplots()
ax = df_n.plot(x='x_n',y='y_n',label="nominal")
df_f.plot(x='x_f',y='y_f',linestyle="--",label="feedback",ax=ax)
legend = ax.legend()
plt.title("Trajectories of nominal and perturbed system with feedback")
plt.savefig("xy_nf.png")
plt.show()


distance = numpy.genfromtxt("atomic_speed_nf.csv", names=["s"])

distancef = numpy.genfromtxt("atomic_theta_nf.csv", names=["t"])

fix, ax = plt.subplots()
ax.plot(range(0,300),distance['s'],label="speed")
ax.plot(range(0,300),distancef['t'],label="direction")
legend = ax.legend()
plt.title("Distance in speed and direction between nominal and perturbed system with feedback")
plt.savefig("distances_nf.png")
plt.show()
