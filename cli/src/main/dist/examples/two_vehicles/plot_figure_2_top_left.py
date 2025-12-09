

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


slow_02 = numpy.genfromtxt("phi_slow_test_02x10.csv", names=["s02t", "s02"])

slow_03 = numpy.genfromtxt("phi_slow_test_03x10.csv", names=["s03t", "s03"])

slow_04 = numpy.genfromtxt("phi_slow_test_04x10.csv", names=["s04t", "s04"])


fix, ax = plt.subplots()
ax.plot(range(0,10),slow_02['s02'],label="0.2")
ax.plot(range(0,10),slow_03['s03'],label="0.3")
ax.plot(range(0,10),slow_04['s04'],label="0.4")
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,10,20,30,40,50,60,70,80,90])
legend = ax.legend(loc='lower right')
plt.title("Evaluation of phi_slow, 10 steps")
plt.savefig("slow_three_values_10.png")
plt.show()

