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

Protein_Z1 = numpy.genfromtxt("new_plotZ1.csv", names=["prot_Z1"])
Protein_Z2 = numpy.genfromtxt("new_plotZ2.csv", names=["prot_Z2"])
Protein_Z3 = numpy.genfromtxt("new_plotZ3.csv", names=["prot_Z3"])

pProtein_Z1 = numpy.genfromtxt("new_pplotZ1.csv", names=["pprot_Z1"])
pProtein_Z2 = numpy.genfromtxt("new_pplotZ2.csv", names=["pprot_Z2"])
pProtein_Z3 = numpy.genfromtxt("new_pplotZ3.csv", names=["pprot_Z3"])

mRNA_X1 = numpy.genfromtxt("new_plotX1.csv", names=["mRNA_X1"])
mRNA_X2 = numpy.genfromtxt("new_plotX2.csv", names=["mRNA_X2"])
mRNA_X3 = numpy.genfromtxt("new_plotX3.csv", names=["mRNA_X3"])

fix, ax = plt.subplots()
ax.plot(range(0,1000),Protein_Z1['prot_Z1'],label="Z1")
ax.plot(range(0,1000),Protein_Z2['prot_Z2'],label="Z2")
ax.plot(range(0,1000),Protein_Z3['prot_Z3'],label="Z3")
plt.title("Evolution of average protein numbers")
legend = ax.legend()
plt.savefig("new_protein.png")
plt.show()

fix, ax = plt.subplots()
ax.plot(range(0,1000),pProtein_Z1['pprot_Z1'],label="Z1")
ax.plot(range(0,1000),pProtein_Z2['pprot_Z2'],label="Z2")
ax.plot(range(0,1000),pProtein_Z3['pprot_Z3'],label="Z3")
plt.title("Evolution of average protein numbers - perturbed case")
legend = ax.legend()
plt.savefig("new_protein.png")
plt.show()

fix, ax = plt.subplots()
ax.plot(range(0,1000),mRNA_X1['mRNA_X1'],label="X1")
ax.plot(range(0,1000),mRNA_X2['mRNA_X2'],label="X2")
ax.plot(range(0,1000),mRNA_X3['mRNA_X3'],label="X3")
legendx = ax.legend()
plt.title("Evolution of average mRNA levels")
plt.savefig("new_mRNA.png")
plt.show()




distance_Z1 = numpy.genfromtxt("atomic_Z1.csv", names=["d_prot_Z1"])
fix, ax = plt.subplots()
line1, = ax.plot(range(0,1000),distance_Z1['d_prot_Z1'],'b:',label="dist_Z1")
ax.set_ylabel('dist_Z1', color='b')

ax1 = ax.twinx()
line2, = ax1.plot(range(0,1000),pProtein_Z1['pprot_Z1'],'g', label="pert_Z1")
line3, = ax1.plot(range(0,1000),Protein_Z1['prot_Z1'],'orange',label="nom_Z1")
ax1.set_ylabel('nom/pert_Z1', color='g')

offset_Z1 = numpy.genfromtxt("offsets_Z1.csv", names=["offsets_prot_Z1"])
plt.title("Evolution of atomic distances")\
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(0,1000),offset_Z1["offsets_prot_Z1"],'r--',label="offset_Z1")
ax2.set_ylabel('offsets_Z1', color='r')



lines = [line1, line2, line3, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper right')

plt.savefig("new_distance.png")
plt.show()


Threshold = []
Value = []

with open('evalR.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        Threshold.append(row[0])
        Value.append(row[1])

plt.scatter(Threshold, Value, color = 'b',s = 100)
plt.xticks(rotation = 0)
plt.xlabel('Threshold')
plt.ylabel('Evaluation (-1.0 = False, 0.0 = Unknown, 1.0 = True)')
plt.title('Robustness', fontsize = 20)

plt.show()

