import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy

pert_10_50 = numpy.genfromtxt("10_50.csv", names=["10"])
pert_250_1000 = numpy.genfromtxt("250_1000.csv", names=["250"])
pert_100_400 = numpy.genfromtxt("100_400.csv", names=["100"])
pert_150_100 = numpy.genfromtxt("150_100.csv", names=["150"])
pert_500_200 = numpy.genfromtxt("500_200.csv", names=["500"])
pert_70_30 = numpy.genfromtxt("500_0.csv", names=["70"])



fix, ax = plt.subplots(figsize=(12,6))
ax.plot(range(0,600),pert_10_50['10'],label="X=10,Y=50")
ax.plot(range(0,600),pert_250_1000['250'],label="X=250,Y=1000")
ax.plot(range(0,600),pert_100_400['100'],label="X=100,Y=400")
legend = ax.legend()
plt.title("Variation of distances with respect to various perturbations")
#plt.savefig("Enzo_distances_yx.png")
plt.show()

fix, ax = plt.subplots(figsize=(12,6))
ax.plot(range(0,600),pert_150_100['150'],label="X=150,Y=100")
ax.plot(range(0,600),pert_500_200['500'],label="X=500,Y=200")
ax.plot(range(0,600),pert_70_30['70'],label="X=500,Y=0")
legend = ax.legend()
plt.title("Variation of distances with respect to various perturbations")
#plt.savefig("Enzo_distances_xy.png")
plt.show()


rob_10_50 = numpy.genfromtxt("rob_25_150.csv", names=["10_50"])
rob_250_1000 = numpy.genfromtxt("rob_250_1000.csv", names=["250_1000"])
rob_100_400 = numpy.genfromtxt("rob_100_400.csv", names=["100_400"])
rob_150_100 = numpy.genfromtxt("rob_150_100.csv", names=["150_100"])
rob_500_200 = numpy.genfromtxt("rob_500_200.csv", names=["500_200"])

fix, ax = plt.subplots()
ax.plot(range(0,10),rob_10_50['10_50'],label="X=10,Y=50")
ax.plot(range(0,10),rob_250_1000['250_1000'],label="X=250,Y=1000")
ax.plot(range(0,10),rob_100_400['100_400'],label="X=100,Y=400")
ax.set_xticks([0,1,2,3,4,5,6,7,8,9,10])
ax.set_xticklabels([0,30,60,90,120,150,180,210,240,270,300])
legend = ax.legend()
plt.title("Variation of robustness with respect to various perturbations")
#plt.savefig("Enzo_formulae_yx.png")
plt.show()

fix, ax = plt.subplots()
ax.plot(range(0,10),rob_150_100['150_100'],label="X=150,Y=100")
ax.plot(range(0,10),rob_500_200['500_200'],label="X=500,Y=200")
ax.set_xticks([0,1,2,3,4,5,6,7,8,9,10])
ax.set_xticklabels([0,30,60,90,120,150,180,210,240,270,300])
legend = ax.legend()
plt.title("Variation of robustness with respect to various perturbations")
#plt.savefig("Enzo_formulae_xy.png")
plt.show()