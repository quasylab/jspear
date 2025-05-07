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
import csv

df_n = pd.read_csv('xy_nominal.csv', names=['x_n', 'y_n'])
df_f_005 = pd.read_csv('xy_feedback_100.csv', names=['x_f_005', 'y_f_005'])
df_f_01 = pd.read_csv('xy_feedback_125.csv', names=['x_f_01', 'y_f_01'])
df_f_015 = pd.read_csv('xy_feedback_150.csv', names=['x_f_015', 'y_f_015'])
df_f_02 = pd.read_csv('xy_feedback_175.csv', names=['x_f_02', 'y_f_02'])
df_f_025 = pd.read_csv('xy_feedback_2.csv', names=['x_f_025', 'y_f_025'])

fix, ax = plt.subplots()
ax = df_f_005.plot(x='x_f_005',y='y_f_005',linestyle="-",label="1.0",ax=ax)
df_f_01.plot(x='x_f_01',y='y_f_01',linestyle="-",label="1.25",ax=ax)
df_f_015.plot(x='x_f_015',y='y_f_015',linestyle="-",label="1.5",ax=ax)
df_f_02.plot(x='x_f_02',y='y_f_02',linestyle="-",label="1.75",ax=ax)
df_f_025.plot(x='x_f_025',y='y_f_025',linestyle="-",label="2.0",ax=ax)
legend = ax.legend()
plt.title("Trajectories of perturbed systems with feedback")
plt.savefig("xy_nf_5.png")
plt.show()
#df_n.plot(x='x_n',y='y_n',label="nominal")

fix, ax = plt.subplots()
ax.plot(range(0,300),df_f_005['x_f_005'],label="1.0")
ax.plot(range(0,300),df_f_01['x_f_01'],label="1.25")
ax.plot(range(0,300),df_f_015['x_f_015'],label="1.5")
ax.plot(range(0,300),df_f_02['x_f_02'],label="1.75")
ax.plot(range(0,300),df_f_025['x_f_025'],label="2.0")
legend = ax.legend()
plt.title("x coordinate in time")
plt.savefig("x_trajectory_nf.png")
plt.show()

fix, ax = plt.subplots()
ax.plot(range(0,300),df_f_005['y_f_005'],label="1.0")
ax.plot(range(0,300),df_f_01['y_f_01'],label="1.25")
ax.plot(range(0,300),df_f_015['y_f_015'],label="1.5")
ax.plot(range(0,300),df_f_02['y_f_02'],label="1.75")
ax.plot(range(0,300),df_f_025['y_f_025'],label="2.0")
legend = ax.legend()
plt.title("y coordinate in time")
plt.savefig("y_trajectory_nf.png")
plt.show()

df_f_15 = pd.read_csv('xy_feedback_15.csv', names=['x_f_15', 'y_f_15'])

fig = plt.figure(figsize=(15,12))
ax = plt.axes(projection='3d')
ax.plot3D(range(0,300),df_f_15['x_f_15'],df_f_15['y_f_15'])
ax.view_init(30, -30)
plt.title("x,y coordinates in time, off=1.5, k=15")
plt.savefig("xy_trajectory_nf.png")
plt.show()

df_f_20 = pd.read_csv('xy_feedback_20.csv', names=['x_f_20', 'y_f_20'])

fig = plt.figure(figsize=(15,12))
ax = plt.axes(projection='3d')
ax.plot3D(range(0,300),df_f_20['x_f_20'],df_f_20['y_f_20'])
ax.view_init(30, -30)
plt.title("x,y coordinates in time, off=1.5, k=20")
plt.savefig("xy_trajectory_nf_2.png")
plt.show()


distance = numpy.genfromtxt("atomic_speed_nf.csv", names=["s"])

distancef = numpy.genfromtxt("atomic_theta_nf.csv", names=["t"])

fix, ax = plt.subplots()
ax.plot(range(0,300),distance['s'],label="speed")
ax.plot(range(0,300),distancef['t'],label="direction")
legend = ax.legend()
plt.title("Distances between nominal and perturbed system with feedback")
plt.savefig("distances_nf.png")
plt.show()

medn = numpy.genfromtxt("get_med_nominal.csv", names=["mn"])

medf05 = numpy.genfromtxt("get_med_feedback_100.csv", names=["mf05"])
medf06 = numpy.genfromtxt("get_med_feedback_125.csv", names=["mf06"])
medf07 = numpy.genfromtxt("get_med_feedback_150.csv", names=["mf07"])
medf08 = numpy.genfromtxt("get_med_feedback_175.csv", names=["mf08"])
medf09 = numpy.genfromtxt("get_med_feedback_2.csv", names=["mf09"])

fix, ax = plt.subplots()
ax.plot(range(0,300),medn['mn'],label="nominal")
ax.plot(range(0,300),medf05['mf05'],label="1.0")
ax.plot(range(0,300),medf06['mf06'],label="1.25")
ax.plot(range(0,300),medf07['mf07'],label="1.5")
ax.plot(range(0,300),medf08['mf08'],label="1.75")
ax.plot(range(0,300),medf09['mf09'],label="2.0")
legend = ax.legend()
plt.title("Medicine delivery, k=5")
plt.savefig("medicine_nf_5.png")
plt.show()

failn = numpy.genfromtxt("fail_nominal.csv", names=["fn"])
failf05 = numpy.genfromtxt("fail_feedback_100.csv", names=["ff05"])
failf06 = numpy.genfromtxt("fail_feedback_125.csv", names=["ff06"])
failf07 = numpy.genfromtxt("fail_feedback_150.csv", names=["ff07"])
failf08 = numpy.genfromtxt("fail_feedback_175.csv", names=["ff08"])
failf09 = numpy.genfromtxt("fail_feedback_2.csv", names=["ff09"])

fix, ax = plt.subplots()
ax.plot(range(0,300),failn['fn'],label="nominal")
ax.plot(range(0,300),failf05['ff05'],label="1.0")
ax.plot(range(0,300),failf06['ff06'],label="1.25")
ax.plot(range(0,300),failf07['ff07'],label="1.5")
ax.plot(range(0,300),failf08['ff08'],label="1.75")
ax.plot(range(0,300),failf09['ff09'],label="2.0")
legend = ax.legend()
plt.title("Dropped medicine, k=5")
plt.savefig("fail_nf_5.png")
plt.show()



medf5 = numpy.genfromtxt("get_med_feedback_5.csv", names=["mf5"])
medf10 = numpy.genfromtxt("get_med_feedback_10.csv", names=["mf10"])
medf15 = numpy.genfromtxt("get_med_feedback_15.csv", names=["mf15"])
medf20 = numpy.genfromtxt("get_med_feedback_20.csv", names=["mf20"])
medf25 = numpy.genfromtxt("get_med_feedback_25.csv", names=["mf25"])
medf30 = numpy.genfromtxt("get_med_feedback_30.csv", names=["mf30"])

fix, ax = plt.subplots()
ax.plot(range(0,300),medf5['mf5'],label="5")
ax.plot(range(0,300),medf10['mf10'],label="10")
ax.plot(range(0,300),medf15['mf15'],label="15")
ax.plot(range(0,300),medf20['mf20'],label="20")
ax.plot(range(0,300),medf25['mf25'],label="25")
ax.plot(range(0,300),medf30['mf30'],label="30")
legend = ax.legend()
plt.title("Medicine delivery, off=1.5")
plt.savefig("medicine_nf_150.png")
plt.show()

failf5 = numpy.genfromtxt("fail_feedback_5.csv", names=["ff5"])
failf10 = numpy.genfromtxt("fail_feedback_10.csv", names=["ff10"])
failf15 = numpy.genfromtxt("fail_feedback_15.csv", names=["ff15"])
failf20 = numpy.genfromtxt("fail_feedback_20.csv", names=["ff20"])
failf25 = numpy.genfromtxt("fail_feedback_25.csv", names=["ff25"])
failf30 = numpy.genfromtxt("fail_feedback_30.csv", names=["ff30"])

fix, ax = plt.subplots()
ax.plot(range(0,300),failf5['ff5'],label="5")
ax.plot(range(0,300),failf10['ff10'],label="10")
ax.plot(range(0,300),failf15['ff15'],label="15")
ax.plot(range(0,300),failf20['ff20'],label="20")
ax.plot(range(0,300),failf25['ff25'],label="25")
ax.plot(range(0,300),failf30['ff30'],label="30")
legend = ax.legend()
plt.title("Dropped medicine, off=1.5")
plt.savefig("fail_nf_150.png")
plt.show()

Threshold = []
Value_125 = []
Value_150 = []
Value_175 = []

with open('FevalR_125.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        Threshold.append(row[0])
        Value_125.append(row[1])
        
with open('FevalR_150.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        Value_150.append(row[1])
        
with open('FevalR_175.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        Value_175.append(row[1])

plt.scatter(Threshold, Value_175, marker='o', color='g', label="1.75")
plt.scatter(Threshold, Value_150, marker='*', color='orange', label="1.50")
plt.scatter(Threshold, Value_125, marker='+', color='b', label="1.25")
plt.legend()
plt.xticks(rotation = 0)
plt.xlabel('Threshold')
plt.ylabel('Evaluation (-1.0 = False, 1.0 = True)')
plt.title('Robustness for k=15')
plt.savefig("rob_fail.png")
plt.show()





