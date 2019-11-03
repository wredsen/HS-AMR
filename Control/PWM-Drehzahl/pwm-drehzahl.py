import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches

# erster Wert linker Encoder, zweiter Wert rechter Encoder
mean_value_left = np.arange(20,dtype=float).reshape((10,2)) # pro Zeile (PWM, Drehzahl)
mean_value_right = np.arange(20,dtype=float).reshape((10,2)) # pro Zeile (PWM, Drehzahl)

i = 10
while i <= 100:
     mean_value_left[(i//10)-1, 0] = i
     mean_value_right[(i//10)-1, 0] = i
     with open("/home/wredi/Desktop/HS-AMR/Control/PWM-Drehzahl/LogDaten/NXTData"+str(i)+".txt","r") as file:
          if file.mode == 'r':
               number_of_measures = 0
               for line in enumerate(file): 
                    if line[1].count(";") > 0:
                         try:
                              number_of_measures += 1

                              k = 1
                              while line[1][-k] != ";":
                                   k+=1
                              right_meas = float(line[1][-(k-1):])
                              mean_value_right[(i//10)-1, 1] += right_meas

                              try: 
                                   m = k+1
                                   while line[1][-m] != ";":
                                        m+=1
                                   left_meas = float(line[1][-(m-1):-(k+1)])
                                   mean_value_left[(i//10)-1, 1] += left_meas
                              except IndexError:
                                   print('m greater array')

                         except ValueError:
                              print('not a valid number')

               mean_value_right[(i//10)-1, 1] = mean_value_right[(i//10)-1, 1] / number_of_measures
               mean_value_left[(i//10)-1, 1] = mean_value_left[(i//10)-1, 1] / number_of_measures
               
     i = i+10

#print(mean_value_right[:,0])
#print(mean_value_left[:,1])
#print(mean_value_right[:,1])
            
          

plt.plot(mean_value_left[:,1], mean_value_left[:,0],'ro')
plt.plot(mean_value_right[:,1], mean_value_right[:,0],'bo')
fit_left = np.polyfit(mean_value_left[:, 1],mean_value_left[:, 0],1)
fit_right = np.polyfit(mean_value_right[:, 1],mean_value_right[:, 0],1)
print("Left-Motor-RPM: "+str(fit_left[0])+"*PWM +"+str(fit_left[1]))
print("Right-Motor-RPM: "+str(fit_right[0])+"*PWM +"+str(fit_right[1]))
func_left = np.poly1d(fit_left) 
func_right = np.poly1d(fit_right) 
plt.plot(mean_value_left[:, 0],func_left(mean_value_left[:, 0]), 'r-')
plt.plot(mean_value_right[:, 0],func_right(mean_value_right[:, 0]), 'b-')
plt.xlabel("RPM [?]")
plt.ylabel("PWM [%]")
red_patch = mpatches.Patch(color='red', label='linker Motor')
blue_patch = mpatches.Patch(color='blue', label='rechter Motor')
plt.legend(handles=[red_patch, blue_patch])
plt.show()
