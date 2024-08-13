#  STARK: Software Tool for the Analysis of Robustness in the unKnown environment
#
#                 Copyright (C) 2023.
#
#  See the NOTICE file distributed with this work for additional information
#  regarding copyright ownership.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#              http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
#  or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean
import csv

mX = numpy.genfromtxt("new_plotLX.csv", names=["l_X"])
mY1 = numpy.genfromtxt("new_plotLY1.csv", names=["l_Y1"])
mY2 = numpy.genfromtxt("new_plotLY2.csv", names=["l_Y2"])
mZ = numpy.genfromtxt("new_plotLZ.csv", names=["l_Z"])

mXpert = numpy.genfromtxt("new_plotLXpert.csv", names=["l_Xpert"])
mY1pert = numpy.genfromtxt("new_plotLY1pert.csv", names=["l_Y1pert"])
mY2pert = numpy.genfromtxt("new_plotLY2pert.csv", names=["l_Y2pert"])
mZpert = numpy.genfromtxt("new_plotLZpert.csv", names=["l_Zpert"])


fix, ax = plt.subplots()

ax.plot(range(0,300),mY1['l_Y1'],label="Y1")
ax.plot(range(0,300),mY2['l_Y2'],label="Y2")
#ax.plot(range(0,300),mY1pert['l_Y1pert'],label="Y1p")
#ax.plot(range(0,300),mY2pert['l_Y2pert'],label="Y2p")

plt.title("Evolution of Y1 and Y2 in the nominal system")
legend = ax.legend()
plt.savefig("new_protein.png")
plt.show()


fix, ax = plt.subplots()

# ax.plot(range(0,300),mY1['l_Y1'],label="Y1")
# ax.plot(range(0,300),mY2['l_Y2'],label="Y2")
ax.plot(range(0,300),mY1pert['l_Y1pert'],label="Y1p")
ax.plot(range(0,300),mY2pert['l_Y2pert'],label="Y2p")

plt.title("Evolution of Y1 and Y2 in the perturbed system")
legend = ax.legend()
plt.savefig("new_protein.png")
plt.show()

fix, ax = plt.subplots()

ax.plot(range(0,300),mY1['l_Y1'],label="Y1")
# ax.plot(range(0,300),mY2['l_Y2'],label="Y2")
ax.plot(range(0,300),mY1pert['l_Y1pert'],label="Y1p")
# ax.plot(range(0,300),mY2pert['l_Y2pert'],label="Y2p")

plt.title("Evolution of Y1 in the nominal and perturbed system")
legend = ax.legend()
plt.savefig("new_protein.png")
plt.show()

fix, ax = plt.subplots()

# ax.plot(range(0,300),mY1['l_Y1'],label="Y1")
ax.plot(range(0,300),mY2['l_Y2'],label="Y2")
# ax.plot(range(0,300),mY1pert['l_Y1pert'],label="Y1p")
ax.plot(range(0,300),mY2pert['l_Y2pert'],label="Y2p")

plt.title("Evolution of Y2 in the nominal and perturbed system")
legend = ax.legend()
plt.savefig("new_protein.png")
plt.show()

fix, ax = plt.subplots()


import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('new_plotLY1Y2.csv', names=['Y1', 'Y2'])

df.plot(x = 'Y1', y = 'Y2', label = 'Evolution of Y1 and Y2 in the nominal system')

#plt.title('Title')

plt.ylabel('Y2')

plt.savefig('trajectory.png')

plt.show()


df = pd.read_csv('new_plotLY1Y2pert.csv', names=['Y1', 'Y2'])

df.plot(x = 'Y1', y = 'Y2', label = 'Evolution of Y1 and Y2 in the perturbed system')

#plt.title('Title')

plt.ylabel('Y2')


plt.savefig('trajectory.png')

plt.show()




fix, ax = plt.subplots()
distance_Y1 = numpy.genfromtxt("new_Latomic_Y1.csv", names=["d_Y1"])
distance_Y2 = numpy.genfromtxt("new_Latomic_Y2.csv", names=["d_Y2"])

ax.plot(range(0,300),distance_Y1['d_Y1'],label="dist_Y1")
ax.plot(range(0,300),distance_Y2['d_Y2'],label="dist_Y2")

legend = ax.legend()
plt.title("Evolution of atomic distances")
plt.savefig("new_distance.png")
plt.show()



