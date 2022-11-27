import numpy as np
import matplotlib.pyplot as plt

#FILL IN THESE VALUES:
obj = np.array([47.4246895,8.3873573])
p1 = np.array([47.4246282,8.3874836])
d = 10 #distance from p1 to obj

#CONSTANTS
R = 6378137 #earth radius in meters


#CALCULATIONS
dir = (obj-p1)/np.linalg.norm(obj-p1)
d_deg = (d*180)/(np.pi*R)
p1_opt = obj - d_deg*dir
p2_opt = obj + d_deg*dir
pfar_opt = obj + 30*d_deg*dir

#PRINT RESULT
print("obj: ["+str(obj[0])+", "+str(obj[1])+"]")
print("p1: ["+str(p1_opt[0])+", "+str(p1_opt[1])+"]")
print("p2: ["+str(p2_opt[0])+", "+str(p2_opt[1])+"]")
print("pfar: ["+str(pfar_opt[0])+", "+str(pfar_opt[1])+"]") #point far away for alignment

#VISUALIZATION
# plt.scatter(obj[0],obj[1],color='green')
# plt.scatter(p1[0],p1[1],color='red')
# plt.scatter(p1_opt[0],p1_opt[1],color='blue')
# plt.scatter(p2_opt[0],p2_opt[1],color='blue')
# plt.show()