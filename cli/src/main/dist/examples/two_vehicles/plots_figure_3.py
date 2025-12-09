

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


speed_005_50 = numpy.genfromtxt("phi_speed_test_005x50.csv", names=["cs00550"])

speed_01_50 = numpy.genfromtxt("phi_speed_test_01x50.csv", names=["cs0150"])

speed_015_50 = numpy.genfromtxt("phi_speed_test_015x50.csv", names=["cs0150"])


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
