import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

fig = plt.figure()
ax = fig.subplots()

dirname, fname = os.path.split(os.path.realpath(__file__))
file_name = "test"

data = pd.read_csv(dirname +"/"+ file_name+".csv")

ax.plot(data['x'],data['z'],marker='x')
ax.quiver(data['x'],data['z'],data['vx'],data['vz'],width=0.001,color='red')
ax.quiver(data['x'],data['z'],np.cos(data['u1_opt']),np.sin(data['u1_opt']),width=0.001,color='blue')
print(data)
# print(data['vz'])
plt.show()