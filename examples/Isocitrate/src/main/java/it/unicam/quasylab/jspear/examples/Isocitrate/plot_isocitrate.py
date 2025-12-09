import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy

data_z1 = numpy.genfromtxt("atomic_001_100.csv", names=["z1"])
data_z2 = numpy.genfromtxt("atomic_001_500.csv", names=["z2"])
data_z3 = numpy.genfromtxt("atomic_10_1000.csv", names=["z3"])


fix, ax = plt.subplots(figsize=(11,6))
ax.plot(range(200,1200),data_z1['z1'],label="E = 0.001, Ip = 100")
ax.plot(range(200,1200),data_z2['z2'],label="E = 0.001, Ip = 500")
ax.plot(range(200,1200),data_z3['z3'],label="E = 10, Ip = 1000")
legend = ax.legend()
plt.title("Variation of the concetration of I wrt different perturbations")
plt.savefig("isocitrate_Nasti.png")
plt.show()

data_z4 = numpy.genfromtxt("atomic_1_500.csv", names=["z4"])
data_z5 = numpy.genfromtxt("atomic_10_100.csv", names=["z5"])
data_z6 = numpy.genfromtxt("atomic_50_300.csv", names=["z6"])


fix, ax = plt.subplots(figsize=(11,6))
ax.plot(range(200,1200),data_z4['z4'],label="E = 1, Ip = 500")
ax.plot(range(200,1200),data_z5['z5'],label="E = 10, Ip = 100")
ax.plot(range(200,1200),data_z6['z6'],label="E = 50, Ip = 300")
legend = ax.legend()
plt.title("Variation of the concetration of I wrt different perturbations")
plt.savefig("isocitrate_nostri.png")
plt.show()

data_z7 = numpy.genfromtxt("long_001_100.csv", names=["z7"])
data_z8 = numpy.genfromtxt("long_001_500.csv", names=["z8"])
data_z9 = numpy.genfromtxt("long_10_1000.csv", names=["z9"])

fix, ax = plt.subplots(figsize=(11,6))
ax.plot(range(0,100),data_z7['z7'],label="E = 1, Ip = 500")
ax.plot(range(0,100),data_z8['z8'],label="E = 10, Ip = 100")
ax.plot(range(0,100),data_z9['z9'],label="E = 50, Ip = 300")
legend = ax.legend()
plt.title("Variation of the concetration of I wrt different perturbations")
plt.savefig("isocitrate_long.png")
plt.show()


