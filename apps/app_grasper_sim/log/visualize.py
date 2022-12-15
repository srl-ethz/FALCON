import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import sys

fig1 = plt.figure()
ax1 = fig1.subplots()

fig2 = plt.figure()
ax2 = fig2.subplots()

dirname, fname = os.path.split(os.path.realpath(__file__))
file_name = sys.argv[1]

data = pd.read_csv(dirname +"/"+ file_name)

# length = 0.4
#zr = data['z_real']
# grip_x = data['x']+length*np.cos(data['u1_opt'] - data['u2_opt']);
# grip_z = data['z']+length*np.sin(data['u1_opt'] - data['u2_opt']);
#print(data)
ax1.plot(data['x'],data['z'],marker='x',color='blue')
ax1.plot(data['xr'],data['zr'],marker='x',color='red')

ax2.plot(data['x'],data['vel'],marker='x',color='blue')
ax2.plot(data['x'],data['vxr'],marker='x',color='red')
#ax.scatter(data['gripx'],data['gripz'],marker='o',color='black')
#ax.scatter(grip_x,grip_z,marker='o',color='black')
# for i in range(0,len(data['x'])):
#     #ax.arrow(data['x'][i],data['z'][i],data['gripx'][i]-data['x'][i],data['gripz'][i]-data['z'][i],color='black')
#     ax.arrow(data['x'][i],data['z'][i],grip_x[i]-data['x'][i],grip_z[i]-data['z'][i],color='black')
    
    # ax.arrow(   data['x'][i] - length*np.cos(data['u1_opt'])[i],
    #             data['z'][i] - length*np.sin(data['u1_opt'])[i],
    #             2*length*np.cos(data['u1_opt'])[i],
    #             2*length*np.sin(data['u1_opt'])[i],color='blue',width=0.03)

ax1.scatter(0,10,marker='x',color='pink')
#ax.quiver(data['x'],data['z'],np.cos(data['u1_opt']),np.sin(data['u1_opt']),pivot='mid',headwidth=0,width=0.005,color='blue')
print(data)

ax1.set(xlim=(-5, 5), ylim=(8, 12))
# print(data['vz'])
plt.show()