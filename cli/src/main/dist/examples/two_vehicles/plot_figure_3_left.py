

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


speed_005 = numpy.genfromtxt("phi_crash_speed_test_005x10.csv", names=["cs005t", "cs005"])

speed_01 = numpy.genfromtxt("phi_crash_speed_test_01x10.csv", names=["cs01t", "cs01"])

speed_015 = numpy.genfromtxt("phi_crash_speed_test_015x10.csv", names=["cs015t", "cs015"])


fix, ax = plt.subplots()
ax.plot(range(0,10),speed_005['cs005'],label="0.05")
ax.plot(range(0,10),speed_01['cs01'],label="0.1")
ax.plot(range(0,10),speed_015['cs015'],label="0.15")
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,10,20,30,40,50,60,70,80,90])
legend = ax.legend(loc='lower right')
plt.title("Evaluation of phi_crash_speed, 10 steps")
plt.savefig("crash_speed_three_values_10.png")
plt.show()

