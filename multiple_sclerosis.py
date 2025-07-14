import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean
import pandas as pd
import csv

data = pd.read_csv("multipleSclerosis.csv", names=['E','R','Er','Rr','E2','R2','ER','ER2'])

fig, ax = plt.subplots(figsize=(15,10))
ax.plot(range(0,2000),data['E'],label="Teff")
ax.plot(range(0,2000),data['R'],label="Treg")
legend = ax.legend()
plt.title("10 samples, 2000 steps, healthy")
plt.savefig("MS_10_2000_h.png")
plt.show()

