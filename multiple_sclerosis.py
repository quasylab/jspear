import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean
import pandas as pd
import csv

data = pd.read_csv("multipleSclerosisOde_sick.csv", names=['E','R','Er','Rr','l','L','E-R'])

fig, ax = plt.subplots(figsize=(15,10))
plt.yscale('log')
ax.plot(range(0,2000),data['E'],label="Teff")
ax.plot(range(0,2000),data['R'],label="Treg")
legend = ax.legend()
plt.title("Cross-regulation unhealthy regime")
plt.savefig("CR_sick.png")
plt.show()

fig, ax = plt.subplots(figsize=(15,10))
plt.yscale('log')
ax.plot(range(0,2000),data['l'],label="reversible damage")
ax.plot(range(0,2000),data['L'],label="irreversible damage")
legend = ax.legend()
plt.title("Tissue damage unhealthy regime")
plt.savefig("Damage_sick.png")
plt.show()

fig, ax = plt.subplots(figsize=(12,7))
plt.yscale('log')
ax.plot(range(0,2000),data['E'],label="Teff")
ax.plot(range(0,2000),data['R'],label="Treg")
ax.plot(range(0,2000),data['l'],label="reversible damage")
ax.plot(range(0,2000),data['L'],label="irreversible damage")
legend = ax.legend()
plt.title("Cross-regulation and damage unhealthy regime")
plt.savefig("CR_D_sick.png")
plt.show()


data_h = pd.read_csv("multipleSclerosisOde.csv", names=['Eh','Rh','Erh','Rrh','lh','Lh','E-Rh'])

fig, ax = plt.subplots(figsize=(15,10))
plt.yscale('log')
ax.plot(range(0,2000),data_h['Eh'],label="Teff")
ax.plot(range(0,2000),data_h['Rh'],label="Treg")
legend = ax.legend()
plt.title("Cross-regulation healthy regime")
plt.savefig("CR_healthy.png")
plt.show()

fig, ax = plt.subplots(figsize=(15,10))
plt.yscale('log')
ax.plot(range(0,2000),data_h['lh'],label="reversible damage")
ax.plot(range(0,2000),data_h['Lh'],label="irreversible damage")
legend = ax.legend()
plt.title("Tissue damage healthy regime")
plt.savefig("Damage_healthy.png")
plt.show()

fig, ax = plt.subplots(figsize=(12,7))
plt.yscale('log')
ax.plot(range(0,2000),data_h['Eh'],label="Teff")
ax.plot(range(0,2000),data_h['Rh'],label="Treg")
ax.plot(range(0,2000),data_h['lh'],label="reversible damage")
ax.plot(range(0,2000),data_h['Lh'],label="irreversible damage")
legend = ax.legend()
plt.title("Cross-regulation and damage healthy regime")
plt.savefig("CR_D_healthy.png")
plt.show()


