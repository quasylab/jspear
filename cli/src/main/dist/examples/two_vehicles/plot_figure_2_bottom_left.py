

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


comb_02 = numpy.genfromtxt("phi_comb_test_02x10.csv", names=["c02t", "c02"])

comb_03 = numpy.genfromtxt("phi_comb_test_03x10.csv", names=["c03t", "c03"])

comb_04 = numpy.genfromtxt("phi_comb_test_04x10.csv", names=["c04t", "c04"])


fix, ax = plt.subplots()
ax.plot(range(0,10),comb_02['c02'],label="0.2")
ax.plot(range(0,10),comb_03['c03'],label="0.3")
ax.plot(range(0,10),comb_04['c04'],label="0.4")
legend = ax.legend(loc='lower right')
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,10,20,30,40,50,60,70,80,90])
plt.title("Evaluation of phi_comb, 10 steps")
plt.savefig("comb_three_values_10.png")
plt.show()

