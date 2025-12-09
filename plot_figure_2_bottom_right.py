

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


comb_02_50 = numpy.genfromtxt("phi_comb_test_02x50.csv", names=["c0250"])

comb_03_50 = numpy.genfromtxt("phi_comb_test_03x50.csv", names=["c0350"])

comb_04_50 = numpy.genfromtxt("phi_comb_test_04x50.csv", names=["c0450"])


fix, ax = plt.subplots()
ax.plot(range(0,10),comb_02_50['c0250'],label="0.2")
ax.plot(range(0,10),comb_03_50['c0350'],label="0.3")
ax.plot(range(0,10),comb_04_50['c0450'],label="0.4")
legend = ax.legend()
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,50,100,150,200,250,300,350,400,450])
plt.title("Evaluation of phi_comb, 50 steps")
plt.savefig("comb_three_values_50.png")
plt.show()

