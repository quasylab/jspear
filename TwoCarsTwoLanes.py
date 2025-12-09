# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 17:50:50 2025

@author: 20235409
"""

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean
import pandas as pd
import csv

my_xy = pd.read_csv('my_trajectory.csv', names=['my_x', 'my_y'])
other_xy = pd.read_csv('other_trajectory.csv', names=['other_x', 'other_y'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy['other_x'],other_xy['other_y'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy['my_x'],my_xy['my_y'],label="controlled",color='b')
legend = ax_3d.legend()
plt.title("Scenario1 single trajectory, cycle=2");
plt.savefig("overtake_trajectory.png")
plt.show()

my_xy_e = pd.read_csv('my_extra_trajectory.csv', names=['my_x_e', 'my_y_e'])
other_xy_e = pd.read_csv('other_extra_trajectory.csv', names=['other_x_e', 'other_y_e'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_e['other_x_e'],other_xy_e['other_y_e'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_e['my_x_e'],my_xy_e['my_y_e'],label="controlled",color='b')
legend = ax_3d.legend()
plt.title("Scenario1 average trajectory, cycle=2");
plt.savefig("overtake_trajectory_e.png")
plt.show()

my_xy_p = pd.read_csv('my_extra_trajectory_p.csv', names=['my_x_p', 'my_y_p'])
other_xy_p = pd.read_csv('other_extra_trajectory_p.csv', names=['other_x_p', 'other_y_p'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_p['other_x_p'],other_xy_p['other_y_p'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_p['my_x_p'],my_xy_p['my_y_p'],label="controlled",color='b')
legend = ax_3d.legend()
plt.title("Scenario1 average perturbed trajectory, cycle=2");
#ax_3d.view_init(10, -130)
plt.savefig("overtake_trajectory_p.png")
plt.show()



my_xy_scen2 = pd.read_csv('my_trajectory_scen2.csv', names=['my_x_2', 'my_y_2'])
other_xy_scen2 = pd.read_csv('other_trajectory_scen2.csv', names=['other_x_2', 'other_y_2'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_scen2['other_x_2'],other_xy_scen2['other_y_2'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_scen2['my_x_2'],my_xy_scen2['my_y_2'],label="controlled",color='b')
legend = ax_3d.legend()
#ax_3d.view_init(10, -130)
plt.title("Scenario2 single trajectory, cycle=2");
plt.savefig("overtake_trajectory_2.png")
plt.show()

my_xy_e2 = pd.read_csv('my_extra_trajectory_scen2.csv', names=['my_x_e2', 'my_y_e2'])
other_xy_e2 = pd.read_csv('other_extra_trajectory_scen2.csv', names=['other_x_e2', 'other_y_e2'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_e2['other_x_e2'],other_xy_e2['other_y_e2'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_e2['my_x_e2'],my_xy_e2['my_y_e2'],label="controlled",color='b')
legend = ax_3d.legend()
plt.title("Scenario2 average trajectory, cycle=2");
plt.savefig("overtake_trajectory_e_2.png")
plt.show()

my_xy_p2 = pd.read_csv('my_extra_trajectory_p_scen2.csv', names=['my_x_p2', 'my_y_p2'])
other_xy_p2 = pd.read_csv('other_extra_trajectory_p_scen2.csv', names=['other_x_p2', 'other_y_p2'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_p2['other_x_p2'],other_xy_p2['other_y_p2'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_p2['my_x_p2'],my_xy_p2['my_y_p2'],label="controlled",color='b')
legend = ax_3d.legend()
#ax_3d.view_init(10, -130)
plt.title("Scenario2 average perturbed trajectory, cycle=2");
plt.savefig("overtake_trajectory_p_2.png")
plt.show()



my_xy_scen3 = pd.read_csv('my_trajectory_scen3.csv', names=['my_x_3', 'my_y_3'])
other_xy_scen3 = pd.read_csv('other_trajectory_scen3.csv', names=['other_x_3', 'other_y_3'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_scen3['other_x_3'],other_xy_scen3['other_y_3'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_scen3['my_x_3'],my_xy_scen3['my_y_3'],label="controlled",color='b')
legend = ax_3d.legend()
plt.title("Scenario3 single trajectory, cycle=2");
plt.savefig("overtake_trajectory_3.png")
plt.show()

my_xy_e3 = pd.read_csv('my_extra_trajectory_scen3.csv', names=['my_x_e3', 'my_y_e3'])
other_xy_e3 = pd.read_csv('other_extra_trajectory_scen3.csv', names=['other_x_e3', 'other_y_e3'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_e3['other_x_e3'],other_xy_e3['other_y_e3'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_e3['my_x_e3'],my_xy_e3['my_y_e3'],label="controlled",color='b')
legend = ax_3d.legend()
plt.title("Scenario3 average trajectory, cycle=2");
plt.savefig("overtake_trajectory_e_3.png")
plt.show()

my_xy_p3 = pd.read_csv('my_extra_trajectory_p_scen3.csv', names=['my_x_p3', 'my_y_p3'])
other_xy_p3 = pd.read_csv('other_extra_trajectory_p_scen3.csv', names=['other_x_p3', 'other_y_p3'])

fig = plt.subplot()
ax_3d = plt.axes(projection="3d")
ax_3d.plot(range(0,300),other_xy_p3['other_x_p3'],other_xy_p3['other_y_p3'],label="other",color='orange')
ax_3d.plot(range(0,300),my_xy_p3['my_x_p3'],my_xy_p3['my_y_p3'],label="controlled",color='b')
legend = ax_3d.legend()
#ax_3d.view_init(10, -130)
plt.title("Scenario3 average perturbed trajectory, cycle=2");
plt.savefig("overtake_trajectory_p_3.png")
plt.show()








atomic_crash = numpy.genfromtxt("atomic_crash_speed.csv", names=["cs"])

fix, ax = plt.subplots()
ax.plot(range(0,300),atomic_crash['cs'])
#legend = ax.legend()
plt.title("Distance with respect to severity of impact, scenario 1, cycle=2")
plt.savefig("Severity of impact.png")
plt.show()

atomic_crash_2 = numpy.genfromtxt("atomic_crash_speed_scen2.csv", names=["cs2"])

fix, ax = plt.subplots()
ax.plot(range(0,300),atomic_crash_2['cs2'])
#legend = ax.legend()
plt.title("Distance with respect to severity of impact, scenario 2, cycle=2")
plt.savefig("Severity of impact_2.png")
plt.show()

atomic_crash_3 = numpy.genfromtxt("atomic_crash_speed_scen3.csv", names=["cs3"])

fix, ax = plt.subplots()
ax.plot(range(0,300),atomic_crash_3['cs3'])
#legend = ax.legend()
plt.title("Distance with respect to severity of impact, scenario 3, cycle=2")
plt.savefig("Severity of impact_3.png")
plt.show()

fix, ax = plt.subplots()
ax.plot(range(0,300),atomic_crash['cs'],label="Scenario 1")
ax.plot(range(0,300),atomic_crash_2['cs2'],label="Scenario 2")
ax.plot(range(0,300),atomic_crash_3['cs3'],label="Scenario 3")
legend = ax.legend()
plt.title("Distance with respect to severity of impact, cycle=5")
plt.savefig("Severity of impact_all_res5.png")
plt.show()



sstep1 = numpy.genfromtxt("step1_crash_speed_scen3.csv", names=["ss1"])
sstep2 = numpy.genfromtxt("step2_crash_speed_scen3.csv", names=["ss2"])
sstep3 = numpy.genfromtxt("step3_crash_speed_scen3.csv", names=["ss3"])
sstep4 = numpy.genfromtxt("step4_crash_speed_scen3.csv", names=["ss4"])
sstep5 = numpy.genfromtxt("step5_crash_speed_scen3.csv", names=["ss5"])


fix, ax = plt.subplots()
ax.plot(range(0,300),sstep1['ss1'],label="Step 1")
ax.plot(range(0,300),sstep2['ss2'],label="Step 2")
ax.plot(range(0,300),sstep3['ss3'],label="Step 3")
ax.plot(range(0,300),sstep4['ss4'],label="Step 4")
ax.plot(range(0,300),sstep5['ss5'],label="Step 5")
legend = ax.legend()
plt.title("Severity of impact, cycle=5")
plt.savefig("speed_scen3_res5.png")
plt.show()


step1 = numpy.genfromtxt("step1_crash_scen3.csv", names=["step1"])
step2 = numpy.genfromtxt("step2_crash_scen3.csv", names=["step2"])
step3 = numpy.genfromtxt("step3_crash_scen3.csv", names=["step3"])
step4 = numpy.genfromtxt("step4_crash_scen3.csv", names=["step4"])
step5 = numpy.genfromtxt("step5_crash_scen3.csv", names=["step5"])


fix, ax = plt.subplots()
ax.plot(range(0,300),step1['step1'],label="Step 1")
ax.plot(range(0,300),step2['step2'],label="Step 2")
ax.plot(range(0,300),step3['step3'],label="Step 3")
ax.plot(range(0,300),step4['step4'],label="Step 4")
ax.plot(range(0,300),step5['step5'],label="Step 5")
legend = ax.legend()
plt.title("Probability of impact, cycle=5")
plt.savefig("crash_scen3_res5.png")
plt.show()





phi_crash = numpy.genfromtxt("three_val_crash.csv", names=["tvc"])
phi_crash_2 = numpy.genfromtxt("three_val_crash_scen2.csv", names=["tvc2"])
phi_crash_3 = numpy.genfromtxt("three_val_crash_scen3.csv", names=["tvc3"])


fix, ax = plt.subplots()
ax.plot(range(0,10),phi_crash['tvc'],marker='o',label="Scenario 1")
ax.plot(range(0,10),phi_crash_2['tvc2'],marker='*',label="Scenario 2")
ax.plot(range(0,10),phi_crash_3['tvc3'],marker='+',label="Scenario 3")
legend = ax.legend()
plt.title("Pointwise evaluation of phi_combined, cycle=2")
plt.savefig("Three_val_phi_crash.png")
plt.show()



