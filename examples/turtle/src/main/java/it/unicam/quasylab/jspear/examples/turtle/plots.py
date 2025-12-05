#  JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
#
#               Copyright (C) 2020.
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

import pandas as pd
import matplotlib.pyplot as plt
import numpy
import csv

df_n = pd.read_csv('Fnew_plotxy.csv', names=['x_n', 'y_n'])
df_p = pd.read_csv('Fnew_pplotxy.csv', names=['x_p', 'y_p'])
df_f = pd.read_csv('Fnew_pfplotxy.csv', names=['x_f', 'y_f'])

fix, ax = plt.subplots()
ax = df_n.plot(x='x_n',y='y_n',label="nominal")
df_p.plot(x='x_p', y='y_p', linestyle=":", label="perturbed",ax=ax)
df_f.plot(x='x_f',y='y_f',linestyle="--",label="feedback",ax=ax)
legend = ax.legend()
plt.title("Trajectories of nominal, perturbed, and perturbed system with feedback")
plt.savefig("xy_npf.png")
plt.show()


df.plot(x = 'x', y = 'y', label = 'Waypoints (1,3),(13,3),(7,7),(23,14),(30,31),(32,40),(35,30)')

plt.title('Path nominal system')

plt.ylabel('y')

plt.savefig('trajectory.png')

plt.show()




df = pd.read_csv('Fnew_pplotxy.csv', names=['x', 'y'])

df.plot(x = 'x', y = 'y', label = 'Waypoints (1,3),(13,3),(7,7),(23,14),(30,31),(32,40),(35,30)')

plt.title('Path perturbed system')

plt.ylabel('y')

plt.savefig('trajectoryp.png')

plt.show()





df.plot(x = 'x', y = 'y', label = 'Waypoints (1,3),(13,3),(7,7),(23,14),(30,31),(32,40),(35,30)')

plt.title('Path perturbed system with feedback')

plt.ylabel('y')

plt.savefig('trajectoryfp.png')

plt.show()

distance = numpy.genfromtxt("Fatomic_P2P.csv", names=["d_P2P"])

distancef = numpy.genfromtxt("Fatomic_FP2P.csv", names=["d_FP2P"])

fix, ax = plt.subplots()
ax.plot(range(0,200),distance['d_P2P'],label="distance")
ax.plot(range(0,200),distancef['d_FP2P'],label="distance")
legend = ax.legend()
plt.title("Evolution distance between nominal and perturbed system")
plt.savefig("new_distance.png")
plt.show()







fix, ax = plt.subplots()
ax.plot(range(0,200),distancef['d_FP2P'],label="distance")
legend = ax.legend()
plt.title("Evolution distance between nominal and perturbed system with feedback")
plt.savefig("new_distance.png")
plt.show()


Threshold = []
Value = []

with open('FevalR.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        Threshold.append(row[0])
        Value.append(row[1])

plt.scatter(Threshold, Value, color = 'b',s = 100)
plt.xticks(rotation = 0)
plt.xlabel('Threshold')
plt.ylabel('Evaluation (-1.0 = False, 0.0 = Unknown, 1.0 = True)')
plt.title('Robustness without feedback', fontsize = 20)

plt.show()


ThresholdF = []
ValueF = []

with open('FevalRF.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        ThresholdF.append(row[0])
        ValueF.append(row[1])

plt.scatter(ThresholdF, ValueF, color = 'b',s = 100)
plt.xticks(rotation = 0)
plt.xlabel('Threshold')
plt.ylabel('Evaluation (-1.0 = False, 0.0 = Unknown, 1.0 = True)')
plt.title('Robustness with feedback', fontsize = 20)

plt.show()
