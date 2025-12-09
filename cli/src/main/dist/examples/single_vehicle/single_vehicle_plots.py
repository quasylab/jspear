import matplotlib.pyplot as plt
import numpy


bool_val_03 = numpy.genfromtxt("phi_slow_03_test_boolean.csv", names=["x","b_03"])
bool_val_04 = numpy.genfromtxt("phi_slow_04_test_boolean.csv", names=["x","b_04"])
bool_val_05 = numpy.genfromtxt("phi_slow_05_test_boolean.csv", names=["x","b_05"])

plt.rcParams["figure.figsize"] = (12,3)

fix, ax = plt.subplots()
ax.plot(range(0,31),bool_val_03['b_03'],label="0.3")
ax.plot(range(0,31),bool_val_04['b_04'],label="0.4")
ax.plot(range(0,31),bool_val_05['b_05'],label="0.5")
ax.set_xticks([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30])
ax.set_xticklabels([0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300])
legend = ax.legend(loc='lower right')
plt.title("Boolean evaluation of phi_slow")
plt.savefig("slow_bool_300.png")
plt.show()


three_val_03 = numpy.genfromtxt("phi_slow_03_test_3val.csv", names=["x","t_03"])
three_val_04 = numpy.genfromtxt("phi_slow_04_test_3val.csv", names=["x","t_04"])
three_val_05 = numpy.genfromtxt("phi_slow_05_test_3val.csv", names=["x","t_05"])

fix, ax = plt.subplots()
ax.plot(range(0,31),three_val_03['t_03'],label="0.3")
ax.plot(range(0,31),three_val_04['t_04'],label="0.4")
ax.plot(range(0,31),three_val_05['t_05'],label="0.5")
ax.set_xticks([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30])
ax.set_xticklabels([0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300])
legend = ax.legend(loc='lower right')
plt.title("Three-valued evaluation of phi_slow")
plt.savefig("slow_3val_300.png")
plt.show()

