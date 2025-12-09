

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


speed_005_50 = numpy.genfromtxt("phi_speed_test_005x50.csv", names=["cs00550t", "cs00550"])

speed_01_50 = numpy.genfromtxt("phi_speed_test_01x50.csv", names=["cs0150t", "cs0150"])

speed_015_50 = numpy.genfromtxt("phi_speed_test_015x50.csv", names=["cs0150t", "cs0150"])


fix, ax = plt.subplots()
ax.plot(range(0,10),speed_005_50['cs00550'],label="0.01")
ax.plot(range(0,10),speed_01_50['cs0150'],label="0.1")
ax.plot(range(0,10),speed_015_50['cs01550'],label="0.15")
legend = ax.legend()
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,50,100,150,200,250,300,350,400,450])
plt.title("Evaluation of phi_crash_speed, 50 steps")
plt.savefig("crash_speed_three_values_50.png")
plt.show()

