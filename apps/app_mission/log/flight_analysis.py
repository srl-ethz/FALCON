import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

fig = plt.figure()
ax = fig.subplots()

dirname, fname = os.path.split(os.path.realpath(__file__))
file_name = "Flight_Sun_Nov_20_12_46_52_2022"

data = pd.read_csv(dirname +"/"+ file_name+".csv")

length = 0.3

ax.plot(data['x'],data['y'])

#ax.quiver(data['x'],data['z'],np.cos(data['u1_opt']),np.sin(data['u1_opt']),pivot='mid',headwidth=0,width=0.005,color='blue')
print(data)

# print(data['vz'])
plt.show()