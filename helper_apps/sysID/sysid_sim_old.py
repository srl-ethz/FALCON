import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd
import numpy as np
import os

fig1 = plt.figure()
ax1 = fig1.subplots()
# fig2 = plt.figure()
# ax2 = fig2.subplots()

dirname, fname = os.path.split(os.path.realpath(__file__))
file_name = "sysID_old"

data = pd.read_csv(dirname +"/"+ file_name+".csv")

# const double cd = 0.6417112299;   // from sysID
# const double cl = 4.752798721;

rho = 1.225
A_tot = 1.2
pitch_offset=0.05984281113
thrust =104.72 * ((data['throttle']*100)**2) * (1-data['vel']/25)
mass = 1.5

F_x = thrust * np.cos(-data['pitch']*np.pi/180+pitch_offset)
F_y = thrust * np.sin(-data['pitch']*np.pi/180+pitch_offset)
F_g = 9.81*np.ones(np.shape(F_x))*mass
vel_2 = data['vel']*data['vel']


def drag_ansatz(x, a):
    return a * x


def lift_ansatz(x, a):
    return a * x


#select right data
idx = np.argwhere(np.array( (data['east'])>50) & (np.array(data['east'])<350) & (np.array(data['north']>40)) )[:,0]

C_drag = (F_x*2)/(A_tot*rho*vel_2)
C_lift = ((F_g-F_y)*2)/(A_tot*rho*vel_2)

C_drag_crop = C_drag[idx]
C_lift_crop = C_lift[idx]
pitch_crop = (data['pitch'][idx]*np.pi/180)+pitch_offset



#curve fit
l_opt,l_cov = curve_fit(lift_ansatz, pitch_crop,C_lift_crop)

d_opt,d_cov = curve_fit(drag_ansatz, pitch_crop,C_drag_crop)

x_lift=np.linspace(0,0.05,200)
y_lift=lift_ansatz(x_lift,*l_opt)

x_drag=np.linspace(0,0.05,200)
y_drag=drag_ansatz(x_drag,*d_opt)

ax1.scatter(pitch_crop,C_lift_crop,s=0.1,color='red',label='$c_{lift}$')
ax1.plot(x_lift,y_lift,color='red')

ax1.scatter(pitch_crop,C_drag_crop,s=0.1,color='blue',label='$c_{drag}$')
ax1.plot(x_drag,y_drag,color='blue')

ax1.set_ylabel('$c_{lift}, c_{drag}$ [-]')
ax1.set_xlabel('Angle of Attack [rad]')
ax1.legend(markerscale=10)

print("====LIFT===")
print(l_opt)
print("===DRAG===")
print(d_opt)

plt.savefig("sysID.svg")
plt.show()