

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


slow_02_30 = numpy.genfromtxt("phi_slow_test_02x30.csv", names=["c0450t", "s0230"])

slow_03_30 = numpy.genfromtxt("phi_slow_test_03x30.csv", names=["s0330t", "s0330"])

slow_04_30 = numpy.genfromtxt("phi_slow_test_04x30.csv", names=["s0430t", "s0430"])

fix, ax = plt.subplots()
ax.plot(range(0,10),slow_02_30['s0230'],label="0.2")
ax.plot(range(0,10),slow_03_30['s0330'],label="0.3")
ax.plot(range(0,10),slow_04_30['s0430'],label="0.4")
legend = ax.legend(loc='lower right')
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,30,60,90,120,150,180,210,240,270])
plt.title("Evaluation of phi_slow, 30 steps")
plt.savefig("slow_three_values_30.png")
plt.show()

