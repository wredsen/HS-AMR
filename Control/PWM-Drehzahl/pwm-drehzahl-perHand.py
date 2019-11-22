import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import os

pwm_array = [20, 30, 40, 50, 60, 70, 80, 90, 100]
mean_value_left = [13, 26, 42, 57, 75, 89, 102, 117, 133]
mean_value_right = [10, 19, 35, 49, 65, 78, 92, 104, 124]

plt.plot(mean_value_left[:], pwm_array[:],'ro')
plt.plot(mean_value_right[:], pwm_array[:],'bo')
fit_left = np.polyfit(mean_value_left[:],pwm_array[:],1)
fit_right = np.polyfit(mean_value_right[:],pwm_array[:],1)
print("Left-Motor-PWM: "+str(fit_left[0])+"*RPM +"+str(fit_left[1]))
print("Right-Motor-PWM: "+str(fit_right[0])+"*RPM +"+str(fit_right[1]))
func_left = np.poly1d(fit_left) 
func_right = np.poly1d(fit_right) 
plt.plot(mean_value_left[:], func_left(mean_value_left[:]), 'r-')
plt.plot(mean_value_right[:], func_right(mean_value_right[:]), 'b-')
plt.xlabel("RPM [1/min]")
plt.ylabel("PWM [%]")
red_patch = mpatches.Patch(color='red', label='linker Motor')
blue_patch = mpatches.Patch(color='blue', label='rechter Motor')
plt.legend(handles=[red_patch, blue_patch])
plt.show()