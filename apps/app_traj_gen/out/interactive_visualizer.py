import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import pandas as pd
import os
import sys

#read in file
dirname, fname = os.path.split(os.path.realpath(__file__))
file_name = sys.argv[1]
data = pd.read_csv(dirname +"/"+ file_name)

#gives back indices of the values that satisfy z0=z and vx0=vx
def get_indices(z,vx):
    idx = np.argwhere( np.array(np.isclose(data['z0'],z)) & np.array(np.isclose(data['vx0'],vx) ))
    return idx[:,0]
 
def speed_to_arrow_length(speed):
    return speed/5-1.5


# Define initial parameters
init_height = 1.0
init_speed = 10.0

#Other Parameters
length = 0.1 #length of plane arrow in plot

# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
#line, = ax.plot(t, f(t, init_height, init_speed))
idx = get_indices(init_height,init_height)
traj, = ax.plot(data['x'][idx],data['z'][idx])
ax.scatter(data['gripx'][idx],data['gripz'][idx],marker='o',color='black')

for i in idx:
    ax.arrow(data['x'][i],data['z'][i],data['gripx'][i]-data['x'][i],data['gripz'][i]-data['z'][i],color='black')
    ax.arrow(   data['x'][i] - length*np.cos(data['u1_opt'])[i],
                data['z'][i] - length*np.sin(data['u1_opt'])[i],
                2*length*np.cos(data['u1_opt'])[i],
                2*length*np.sin(data['u1_opt'])[i],color='blue',width=0.03)


arrow = ax.arrow(-2-speed_to_arrow_length(init_speed),init_height,speed_to_arrow_length(init_speed),0,color='red',width=0.01)
object = ax.scatter(0,1,marker='x',s=4)
ax.set_xlabel('x [m]')
ax.set_xlim(-3,2)
ax.set_ylim(0,2.5)

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.25, bottom=0.25)

# Make a horizontal slider to control the speed.
aspeed = fig.add_axes([0.25, 0.1, 0.65, 0.03])
speed_slider = Slider(
    ax=aspeed,
    label='Velocity',
    valmin=9,
    valstep=0.25,
    valmax=12.25,
    valinit=init_speed,
)
# Make a vertically oriented slider to control the height
axheight = fig.add_axes([0.1, 0.25, 0.0225, 0.63])
height_slider = Slider(
    ax=axheight,
    label="Height",
    valmin=0.7,
    valstep=0.05,
    valmax=1.3,
    valinit=init_height,
    orientation="vertical"
)


# The function to be called anytime a slider's value changes
def update(val):
    ax.cla()
    ax.set_xlim(-3,2)
    ax.set_ylim(0,2.5)
    idx = get_indices(height_slider.val,speed_slider.val)
    ax.plot(data['x'][idx],data['z'][idx])
    ax.scatter(data['gripx'][idx],data['gripz'][idx],marker='o',color='black')
    ax.scatter(0,1,marker='x',s=4)
    for i in idx:
        ax.arrow(data['x'][i],data['z'][i],data['gripx'][i]-data['x'][i],data['gripz'][i]-data['z'][i],color='black')
        ax.arrow(   data['x'][i] - length*np.cos(data['u1_opt'])[i],
                data['z'][i] - length*np.sin(data['u1_opt'])[i],
                2*length*np.cos(data['u1_opt'])[i],
                2*length*np.sin(data['u1_opt'])[i],color='blue',width=0.03)


    #line.set_ydata(f(t, height_slider.val, speed_slider.val))
    #ax.texts.remove(arrow)
    ax.arrow(-2-speed_to_arrow_length(speed_slider.val),height_slider.val,speed_to_arrow_length(speed_slider.val),0,color='red',width=0.01)
    fig.canvas.draw_idle()


# register the update function with each slider
speed_slider.on_changed(update)
height_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')


def reset(event):
    speed_slider.reset()
    height_slider.reset()
button.on_clicked(reset)

plt.show()