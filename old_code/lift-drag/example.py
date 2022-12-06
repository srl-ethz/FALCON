import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

lift_angles = np.linspace(0,22,12)
lift_rad = lift_angles * np.pi/180
lift = np.array([0.0, 0.018, 0.031, 0.05, 0.065, 0.08, 0.1, 0.115, 0.13, 0.142, 0.15, 0.12 ])

drag_angles = np.linspace(0,18,10)
drag_rad = drag_angles * np.pi/180
drag = np.array([0.02,0.02,0.03,0.04,0.055,0.07,0.1,0.128,0.17,0.22])


def drag_ansatz(x, a,b):
    return a * x**2+b

def lift_ansatz(x, a,b,c,d,e,f):
    return a * x**5 + b* x**4 + c* x**3 + d*x**2+e*x+f

def lift_ansatz_simple(x, a,b):
    return a * x+b


l_opt,l_cov = curve_fit(lift_ansatz, lift_rad, lift)

l_opt_simple,l_cov_simple = curve_fit(lift_ansatz_simple, lift_rad[0:11], lift[0:11])


x_lift=np.linspace(0,24 * np.pi/180,200)
y_lift=lift_ansatz(x_lift,*l_opt)
y_lift_simple=lift_ansatz_simple(x_lift,*l_opt_simple)

d_opt,d_cov = curve_fit(drag_ansatz, drag_rad, drag)

x_drag=np.linspace(0,20 * np.pi/180,200)
y_drag=drag_ansatz(x_drag,*d_opt)


ax.scatter(lift_rad,lift)
ax.scatter(drag_rad,drag)

ax.plot(x_lift,y_lift_simple)
ax.plot(x_lift,y_lift)
ax.plot(x_drag,y_drag)

print("=======PARAMETERS=======")
print("lift: " + str(l_opt_simple))
print("drag: " + str(d_opt))

plt.show()
