

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean

TEMP_off = numpy.genfromtxt("testTemperature.csv", names=["temp_off"])
TEMP_off1 = numpy.genfromtxt("testTemperature_l1.csv", names=["temp_off1"])
TEMP_off2 = numpy.genfromtxt("testTemperature_l2.csv", names=["temp_off2"])

fix, ax = plt.subplots()
ax.plot(range(90,300),TEMP_off1['temp_off1'],label="o = -1")
ax.plot(range(90,300),TEMP_off['temp_off'],label="o = -1.5")
ax.plot(range(90,300),TEMP_off2['temp_off2'],label="o = -2")
legend = ax.legend()
plt.title("Variation of temperature wrt different offset intervals")
plt.savefig("new_temperature.png")
plt.show()

STRESS_off = numpy.genfromtxt("testStress.csv", names=["stress_off"])
STRESS_off1 = numpy.genfromtxt("testStress_l1.csv", names=["stress_off1"])
STRESS_off2 = numpy.genfromtxt("testStress_l2.csv", names=["stress_off2"])

fix, ax = plt.subplots()
ax.plot(range(90,220),STRESS_off1['stress_off1'],label="o = -1")
ax.plot(range(90,220),STRESS_off['stress_off'],label="o = -1.5")
ax.plot(range(90,220),STRESS_off2['stress_off2'],label="o = -2")
legend = ax.legend()
plt.title("Variation of stress wrt different offset intervals")
plt.savefig("new_stress.png")
plt.show()

WRN_max = numpy.genfromtxt("testIntervalWarn.csv", names=["wrn_max"])
STRESS_max = numpy.genfromtxt("testIntervalSt.csv", names=["stress_max"])

plt.plot(range(0,50),WRN_max['wrn_max'])
plt.title("Evaluation of distances wrt warning over time")
plt.savefig("new_time_wrn.png")
plt.show()

plt.plot(range(0,50),STRESS_max['stress_max'])
plt.title("Evaluation of distances wrt stress over time")
plt.savefig("new_time_stress.png")
plt.show()



CI_left = numpy.genfromtxt("testBootstrapL_50.csv", names=["CIleft"])
CI_right = numpy.genfromtxt("testBootstrapR_50.csv", names=["CIright"])


CI_width = abs(CI_left['CIleft'] - CI_right['CIright'])
CI_mean = sum(CI_width)/len(CI_width)

print("Maximal difference for m=50 "+str(max(CI_width)))
print("Average difference for m=50 "+str(CI_mean))

fix, ax = plt.subplots()
ax.plot(range(0,50),CI_left['CIleft'],label="CI_l")
ax.plot(range(0,50),CI_right['CIright'],label="CI_r")
legend = ax.legend()
plt.title("Evaluation of confidence interval over time")
plt.savefig("new_CI.png")
plt.show()

CI_left_100 = numpy.genfromtxt("testBootstrapL_100.csv", names=["CIleft_100"])
CI_right_100 = numpy.genfromtxt("testBootstrapR_100.csv", names=["CIright_100"])

CI_width_100 = abs(CI_left_100['CIleft_100'] - CI_right_100['CIright_100'])
CI_mean_100 = sum(CI_width_100)/len(CI_width_100)

print("Maximal difference for m=100 "+str(max(CI_width_100)))
print("Average difference for m=100 "+str(CI_mean_100))


fix, ax = plt.subplots()
ax.plot(range(0,50),CI_left_100['CIleft_100'],label="CI_l")
ax.plot(range(0,50),CI_right_100['CIright_100'],label="CI_r")
legend = ax.legend()
plt.title("Evaluation of confidence interval over time")
plt.savefig("new_CI_100.png")
plt.show()

valuation_1 = numpy.genfromtxt("new_testThreeValue1.csv", names=["val1"])
valuation_2 = numpy.genfromtxt("new_testThreeValue2.csv", names=["val2"])
valuation_3 = numpy.genfromtxt("additional_testThreeValue.csv", names=["val3"])
valuation_4 = numpy.genfromtxt("new_testThreeValue3.csv", names=["val4"])

fix, ax = plt.subplots()
ax.plot(range(0,50),valuation_4['val4'],label="eta_3=0.06")
ax.plot(range(0,50),valuation_3['val3'],label="eta_3=0.05")
ax.plot(range(0,50),valuation_2['val2'],label="eta_3=0.04")
ax.plot(range(0,50),valuation_1['val1'],label="eta_3=0.03")
legend = ax.legend()
plt.title("Evaluation of three-valued semantics")
plt.savefig("new_three_values.png")
plt.show()
