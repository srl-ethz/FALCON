import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd
import numpy as np
import os

fig1 = plt.figure()
ax1 = fig1.subplots()
fig2 = plt.figure()
ax2 = fig2.subplots()


dirname, fname = os.path.split(os.path.realpath(__file__))
file_name = "pitch_at_10"

data = pd.read_csv(dirname +"/"+ file_name+".csv")


rho = 1.225
A_tot = 1
pitch_offset=0.05984281113
pitch_offset_deg=pitch_offset*180.0/np.pi
thrust = (8.54858 * 10**(-6) * (3500 * data['throttle'])**2 ) * (1-data['vel']/25)
mass = 1.5


def drag_ansatz(x, a,b):
    return a * x**2+b


def lift_ansatz(x, a,b):
    return a * x+b

# select right data
idx = np.argwhere( (np.array(data['vel'])<10.5) & (np.array(data['vel'])>9.5) )[:,0]

print(np.mean(data['pitch'][idx])*np.pi/180.0)

ax2.scatter(data['t'][idx],data['vel'][idx],s=0.1)


# ax2.scatter(data['east'][idx],vel_angle-data['pitch'][idx], s=0.1)

# vel_2 = data['vel'][idx]*data['vel'][idx] 
# F_x = thrust[idx] * np.cos(data['pitch'][idx]*np.pi/180+pitch_offset)
# F_y = thrust[idx] * np.sin(data['pitch'][idx]*np.pi/180+pitch_offset)
# F_g = 9.81*np.ones(np.shape(F_x))*mass

# print(np.shape(vel_2))
# print(np.shape(F_x))
# C_drag = (F_x*2)/(rho*vel_2)
# C_lift = ((F_g-F_y)*2)/(rho*vel_2)


# pitch_crop = (data['pitch'][idx]*np.pi/180)+pitch_offset

# print(np.shape(pitch_crop))


# # #curve fit
# # l_opt,l_cov = curve_fit(lift_ansatz, pitch_crop,C_lift_crop)

# # d_opt,d_cov = curve_fit(drag_ansatz, pitch_crop,C_drag_crop)

# # x_lift=np.linspace(0,0.05,200)
# # y_lift=lift_ansatz(x_lift,*l_opt)

# # x_drag=np.linspace(0,0.05,200)
# # y_drag=drag_ansatz(x_drag,*d_opt)

# ax1.scatter(pitch_crop,C_lift,color='red')
# # # ax1.plot(x_lift,y_lift,color='red')

# ax1.scatter(pitch_crop,C_drag,color='blue')
# # ax1.plot(x_drag,y_drag,color='blue')

# print("====LIFT===")
# print(l_opt)
# print("===DRAG===")
# print(d_opt)

plt.show()