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

#  STARK: Software Tool for the Analysis of Robustness in the unKnown environment
#
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

distance_Z1 = numpy.genfromtxt("skorokhod_Z1.csv", names=["d_prot_Z1"])
distance_Z2 = numpy.genfromtxt("skorokhod_Z2.csv", names=["d_prot_Z2"])
distance_Z3 = numpy.genfromtxt("skorokhod_Z3.csv", names=["d_prot_Z3"])

adistance_Z1 = numpy.genfromtxt("atomic_Z1.csv", names=["ad_prot_Z1"])
adistance_Z2 = numpy.genfromtxt("atomic_Z2.csv", names=["ad_prot_Z2"])
adistance_Z3 = numpy.genfromtxt("atomic_Z3.csv", names=["ad_prot_Z3"])

# skorokhod distances z1 z2 z3
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],label="distance_Z1")
ax.plot(range(550,1000),distance_Z2['d_prot_Z2'][550:1000],label="distance_Z2")
ax.plot(range(550,1000),distance_Z3['d_prot_Z3'][550:1000],label="distance_Z3")
legend = ax.legend(loc='upper right')
plt.savefig("distances.png")
plt.savefig("distances.eps", format='eps')
plt.show()


# atomic vs skorokhod Z1
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z1['ad_prot_Z1'][550:1000],label="atomic")
legend = ax.legend(loc='upper right')
plt.savefig("SkorokhodAtomicDistancesZ1.png")
plt.savefig("SkorokhodAtomicDistancesZ1.eps", format='eps')

# atomic vs skorokhod Z2
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z2['d_prot_Z2'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z2['ad_prot_Z2'][550:1000],label="atomic")
legend = ax.legend(loc='upper right')
plt.savefig("SkorokhodAtomicDistancesZ2.png")
plt.savefig("SkorokhodAtomicDistancesZ2.eps", format='eps')

# atomic vs skorokhod Z3
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z3['d_prot_Z3'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z3['ad_prot_Z3'][550:1000],label="atomic")
legend = ax.legend(loc='upper right')
plt.savefig("SkorokhodAtomicDistancesZ3.png")
plt.savefig("SkorokhodAtomicDistancesZ3.eps", format='eps')



# offset_sk = numpy.genfromtxt("offsets_SK.csv", names=["offsets_SK"])
# dist_sk = numpy.genfromtxt("rskor.csv", names=["distance"])

# fix, ax = plt.subplots()
# line1, = ax.plot(range(550,1000),dist_sk['distance'][550:1000],'b:',label="distance")
# ax.set_ylabel('distance', color='b')
# # create y-axis that shares x-axis
# ax2 = ax.twinx()
# line4, = ax2.plot(range(550,1000),offset_sk["offsets_SK"],'r--',label="offsets_SK")
# ax2.set_ylabel('offsets_SK', color='r')
# lines = [line1, line4]
# labels = [line.get_label() for line in lines]
# ax.legend(lines, labels, loc='upper left')
# plt.savefig("distanceSK.png")
# plt.savefig("distanceSK.eps", format='eps')

# skorokhod vs skorokhod Z1
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z1['ad_prot_Z1'][550:1000],label="atomic")
# ax.plot(range(550,1000),dist_sk['distance'][550:1000],label="SkorokhodSK")
legend = ax.legend(loc='upper right')
plt.savefig("SkorokhodSkorokhodDistancesZ1.png")
plt.savefig("SkorokhodSkorokhodDistancesZ1.eps", format='eps')



offset_Z1 = numpy.genfromtxt("offsets_Z1.csv", names=["offsets_prot_Z1"])
offset_Z2 = numpy.genfromtxt("offsets_Z2.csv", names=["offsets_prot_Z2"])
offset_Z3 = numpy.genfromtxt("offsets_Z3.csv", names=["offsets_prot_Z3"])

# DistancesZ1 w offsets
fix, ax = plt.subplots()
line1, = ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],'b:',label="distance_Z1")
ax.set_ylabel('distance_Z1', color='b')
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(550,1000),offset_Z1["offsets_prot_Z1"][550:1000],'r--',label="offset_Z1")
ax2.set_ylabel('offsets_Z1', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig("distanceZ1.png")
plt.savefig("distanceZ1.eps", format='eps')

# DistancesZ2 w offsets
fix, ax = plt.subplots()
line1, = ax.plot(range(550,1000),distance_Z2['d_prot_Z2'][550:1000],'b:',label="distance_Z2")
ax.set_ylabel('distance_Z2', color='b')
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(550,1000),offset_Z2["offsets_prot_Z2"][550:1000],'r--',label="offset_Z2")
ax2.set_ylabel('offsets_Z2', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig("distanceZ2.png")
plt.savefig("distanceZ2.eps", format='eps')

# DistancesZ3 w offsets
fix, ax = plt.subplots()
line1, = ax.plot(range(550,1000),distance_Z3['d_prot_Z3'][550:1000],'b:',label="distance_Z3")
ax.set_ylabel('distance_Z2', color='b')
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(550,1000),offset_Z3["offsets_prot_Z3"][550:1000],'r--',label="offset_Z3")
ax2.set_ylabel('offsets_Z3', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig("distanceZ3.png")
plt.savefig("distanceZ3.eps", format='eps')

Protein_Z1 = numpy.genfromtxt("new_plotZ1.csv", names=["prot_Z1"])
Protein_Z2 = numpy.genfromtxt("new_plotZ2.csv", names=["prot_Z2"])
Protein_Z3 = numpy.genfromtxt("new_plotZ3.csv", names=["prot_Z3"])

pProtein_Z1 = numpy.genfromtxt("new_pplotZ1.csv", names=["pprot_Z1"])
pProtein_Z2 = numpy.genfromtxt("new_pplotZ2.csv", names=["pprot_Z2"])
pProtein_Z3 = numpy.genfromtxt("new_pplotZ3.csv", names=["pprot_Z3"])

# Z1 states
fix, ax = plt.subplots()
line2, = ax.plot(range(0,1000),pProtein_Z1['pprot_Z1'],'g', label="perturbed_Z1")
line3, = ax.plot(range(0,1000),Protein_Z1['prot_Z1'],'orange',label="nominal_Z1")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='lower right')
plt.savefig("statesZ1.png")
plt.savefig("statesZ1.eps", format='eps')

# Z2 states
fix, ax = plt.subplots()
line2, = ax.plot(range(0,1000),pProtein_Z2['pprot_Z2'],'g', label="perturbed_Z2")
line3, = ax.plot(range(0,1000),Protein_Z2['prot_Z2'],'orange',label="nominal_Z2")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='lower right')
plt.savefig("statesZ2.png")
plt.savefig("statesZ2.eps", format='eps')

# Z3 states
fix, ax = plt.subplots()
line2, = ax.plot(range(0,1000),pProtein_Z3['pprot_Z3'],'g', label="perturbed_Z3")
line3, = ax.plot(range(0,1000),Protein_Z3['prot_Z3'],'orange',label="nominal_Z3")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='lower right')
plt.savefig("statesZ3.png")
plt.savefig("statesZ3.eps", format='eps')


distance_Z1 = numpy.genfromtxt("skorokhod_Z1.csv", names=["d_prot_Z1"])
fix, ax = plt.subplots()
line1, = ax.plot(range(0,1000),distance_Z1['d_prot_Z1'],'b:',label="distance_Z1")
ax.set_ylabel('distance_Z1', color='b')
# create y-axis that shares x-axis
ax1 = ax.twinx()
line2, = ax1.plot(range(0,1000),pProtein_Z1['pprot_Z1'],'g', label="perturbed_Z1")
line3, = ax1.plot(range(0,1000),Protein_Z1['prot_Z1'],'orange',label="nominal_Z1")
ax1.set_ylabel('nom/perturbed_Z1', color='g')
offset_Z1 = numpy.genfromtxt("offsets_Z1.csv", names=["offsets_prot_Z1"])
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(0,1000),offset_Z1["offsets_prot_Z1"],'r--',label="offset_Z1")
ax2.set_ylabel('offsets_Z1', color='r')
lines = [line1, line2, line3, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper right')

plt.savefig("fullZ1.png")
plt.savefig("fullZ1.eps", format='eps')
