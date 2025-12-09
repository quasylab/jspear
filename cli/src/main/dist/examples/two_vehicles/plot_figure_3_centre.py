

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


speed_005_30 = numpy.genfromtxt("phi_crash_speed_test_005x30.csv", names=["cs00530t", "cs00530"])

speed_01_30 = numpy.genfromtxt("phi_crash_speed_test_01x30.csv", names=["cs0130t", "cs0130"])

speed_015_30 = numpy.genfromtxt("phi_crash_speed_test_015x30.csv", names=["cs01530t", "cs01530"])

fix, ax = plt.subplots()
ax.plot(range(0,10),speed_005_30['cs00530'],label="0.05")
ax.plot(range(0,10),speed_01_30['c0130'],label="0.1")
ax.plot(range(0,10),speed_015_30['c01530'],label="0.15")
legend = ax.legend(loc='lower right')
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,30,60,90,120,150,180,210,240,270])
plt.title("Evaluation of phi_crash_speed, 30 steps")
plt.savefig("crash_speed_three_values_30.png")
plt.show()

