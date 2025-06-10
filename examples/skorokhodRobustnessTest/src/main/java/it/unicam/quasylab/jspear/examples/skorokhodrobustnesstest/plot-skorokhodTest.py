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


distance_Z1 = numpy.genfromtxt("distance.csv", names=["d_prot_Z1"])
fix, ax = plt.subplots()
line1, = ax.plot(range(0,500),distance_Z1['d_prot_Z1'],'b:',label="distance")
ax.set_ylabel('Skorokhod distance', color='b')
offset_Z1 = numpy.genfromtxt("offsets.csv", names=["offsets_prot_Z1"])
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(0,500),offset_Z1["offsets_prot_Z1"],'r--',label="offset")
ax2.set_ylabel('offset', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig("distance.png")
plt.savefig("distance.eps")
plt.show()


Protein_Z1 = numpy.genfromtxt("plotx.csv", names=["prot_Z1"])
pProtein_Z1 = numpy.genfromtxt("pplotx.csv", names=["pprot_Z1"])
fix, ax = plt.subplots()
line2, = ax.plot(range(0,500),pProtein_Z1['pprot_Z1'],'g', label="perturbed")
line3, = ax.plot(range(0,500),Protein_Z1['prot_Z1'],'orange',label="nominal")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig("states.png")
plt.savefig("states.eps")


# HIGH FREQUENCY VERSION

distance_Z1 = numpy.genfromtxt("distanceHF.csv", names=["d_prot_Z1"])
fix, ax = plt.subplots()
line1, = ax.plot(range(0,500),distance_Z1['d_prot_Z1'],'b:',label="distance")
ax.set_ylabel('Skorokhod distance', color='b')
offset_Z1 = numpy.genfromtxt("offsetsHF.csv", names=["offsets_prot_Z1"])
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(0,500),offset_Z1["offsets_prot_Z1"],'r--',label="offset")
ax2.set_ylabel('offset', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig("distanceHF.png")
plt.savefig("distanceHF.eps")

pProtein_Z1 = numpy.genfromtxt("pplotxHF.csv", names=["pprot_Z1HF"])


fix, ax = plt.subplots()
line2, = ax.plot(range(0,500),pProtein_Z1['pprot_Z1HF'],'g', label="perturbed")
line3, = ax.plot(range(0,500),Protein_Z1['prot_Z1'],'orange',label="nominal")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig("statesHF.png")
plt.savefig("statesHF.eps")
