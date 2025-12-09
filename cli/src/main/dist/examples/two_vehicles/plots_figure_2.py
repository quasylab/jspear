

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean


slow_02 = numpy.genfromtxt("phi_slow_test_02x10.csv", names=["s02t", "s02"])
slow_02_30 = numpy.genfromtxt("phi_slow_test_02x30.csv", names=["s0230t", "s0230"])
slow_02_50 = numpy.genfromtxt("phi_slow_test_02x50.csv", names=["s0250t", "s0250"])

slow_03 = numpy.genfromtxt("phi_slow_test_03x10.csv", names=["s03t", "s03"])
slow_03_30 = numpy.genfromtxt("phi_slow_test_03x30.csv", names=["s0330t", "s0330"])
slow_03_50 = numpy.genfromtxt("phi_slow_test_03x50.csv", names=["s0350t", "s0350"])

slow_04 = numpy.genfromtxt("phi_slow_test_04x10.csv", names=["s04t", "s04"])
slow_04_30 = numpy.genfromtxt("phi_slow_test_04x30.csv", names=["s0430t", "s0430"])
slow_04_50 = numpy.genfromtxt("phi_slow_test_04x50.csv", names=["s0450t", "s0450"])


comb_02 = numpy.genfromtxt("phi_comb_test_02x10.csv", names=["c02t", "c02"])
comb_02_30 = numpy.genfromtxt("phi_comb_test_02x30.csv", names=["c0230t", "c0230"])
comb_02_50 = numpy.genfromtxt("phi_comb_test_02x50.csv", names=["c0250t", "c0250"])

comb_03 = numpy.genfromtxt("phi_comb_test_03x10.csv", names=["c03t", "c03"])
comb_03_30 = numpy.genfromtxt("phi_comb_test_03x30.csv", names=["c0330t", "c0330"])
comb_03_50 = numpy.genfromtxt("phi_comb_test_03x50.csv", names=["c0350t", "c0350"])

comb_04 = numpy.genfromtxt("phi_comb_test_04x10.csv", names=["c04t", "c04"])
comb_04_30 = numpy.genfromtxt("phi_comb_test_04x30.csv", names=["c0430t", "c0430"])
comb_04_50 = numpy.genfromtxt("phi_comb_test_04x50.csv", names=["c0450t", "c0450"])


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

fix, ax = plt.subplots()
ax.plot(range(0,10),slow_02_50['s0250'],label="0.2")
ax.plot(range(0,10),slow_03_50['s0350'],label="0.3")
ax.plot(range(0,10),slow_04_50['s0450'],label="0.4")
legend = ax.legend()
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,50,100,150,200,250,300,350,400,450])
plt.title("Evaluation of phi_slow, 50 steps")
plt.savefig("slow_three_values_50.png")
plt.show()


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

fix, ax = plt.subplots()
ax.plot(range(0,10),comb_02_30['c0230'],label="0.2")
ax.plot(range(0,10),comb_03_30['c0330'],label="0.3")
ax.plot(range(0,10),comb_04_30['c0430'],label="0.4")
legend = ax.legend(loc='lower right')
ax.set_xticks([0,1,2,3,4,5,6,7,8,9])
ax.set_xticklabels([0,30,60,90,120,150,180,210,240,270])
plt.title("Evaluation of phi_comb, 30 steps")
plt.savefig("comb_three_values_30.png")
plt.show()

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

