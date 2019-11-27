import numpy as np                      
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import os

dirname = os.path.dirname(__file__)               #get absulute file path

# erster Wert linker Encoder, zweiter Wert rechter Encoder
mean_value_left = np.arange(20,dtype=float).reshape((10,2)) # pro Zeile (PWM, Drehzahl)
mean_value_right = np.arange(20,dtype=float).reshape((10,2)) # pro Zeile (PWM, Drehzahl)

i = 10
while i <= 100:
     mean_value_left[(i//10)-1, 0] = i            #fill array with x values from index
     mean_value_right[(i//10)-1, 0] = i    
     filename = os.path.join(dirname,"LogDaten_mitLast/NXTData"+str(i)+".txt")       #define relative path
     with open(filename,"r") as file:                                                #open file from relative path
          if file.mode == 'r':                                                       #check if file is open as "read"
               number_of_measures = 0                                                #counter for sampeled values
               for line in enumerate(file):                                          #itterarate thru line of file 
                    if line[1].count(";") > 0:                                       #count if ";" is in the tupel line line[0] is the line number
                         try:                                                        #exception for stirng to float conversion 
                              number_of_measures += 1                                #add one sample to the array (counter)

                              k = 1
                              while line[1][-k] != ";":                              #start itterate backwards to search for first ";"
                                   k+=1
                                   
                              right_meas = float(line[1][-(k-1):])                   #cast from string to float line from -(k-1) to end of line
                              mean_value_right[(i//10)-1, 1] += right_meas           #save sample to array 

                         except ValueError:            
                              print('not a valid number')

                         try:                                                        #exception for stirng to float conversion 
                              m = k+1

                              while line[1][-m] != ";":                              #start itterate backwards to search for first ";" beginning from k+1
                                   m+=1

                              left_meas = float(line[1][-(m-1):-(k+1)])              #cast from string to float line from -(m-1) to -(k-1) second sample 
                              mean_value_left[(i//10)-1, 1] += left_meas             #save sample to array

                         except ValueError:
                              print('not a valid number')

               mean_value_right[(i//10)-1, 1] = mean_value_right[(i//10)-1, 1] / number_of_measures      #calculate the mean of the sampeled values
               mean_value_left[(i//10)-1, 1] = mean_value_left[(i//10)-1, 1] / number_of_measures
               
     i = i+10

#print(mean_value_right[:,0])
#print(mean_value_left[:,1])
#print(mean_value_right[:,1])


#Skaliere die RPMs:
sample_time = 0.104 # miliseconds
mean_value_left[:,1] = (mean_value_left[:,1]/sample_time) * (60/360) 
mean_value_right[:,1] = (mean_value_right[:,1]/sample_time) * (60/360)           

plt.plot(mean_value_left[:,1], mean_value_left[:,0],'ro')                            #plot scatter of left motor with red dots  
plt.plot(mean_value_right[:,1], mean_value_right[:,0],'bo')                          #plot scatter of right motor with blue dots

#claculate a fit of first degree
fit_left = np.polyfit(mean_value_left[:, 1],mean_value_left[:, 0],1)                 
fit_right = np.polyfit(mean_value_right[:, 1],mean_value_right[:, 0],1)

#print out function
print("Left-Motor-RPM: "+str(fit_left[0])+"*PWM +"+str(fit_left[1]))                 
print("Right-Motor-RPM: "+str(fit_right[0])+"*PWM +"+str(fit_right[1]))

#generate plottable function
func_left = np.poly1d(fit_left)                                                     
func_right = np.poly1d(fit_right) 

#plot both fit functions 
plt.plot(mean_value_left[:, 1],func_left(mean_value_left[:, 1]), 'r-')
plt.plot(mean_value_right[:, 1],func_right(mean_value_right[:, 1]), 'b-')

#label axes
plt.xlabel("RPM [1/min]")
plt.ylabel("PWM [%]")

#label graphs
red_patch = mpatches.Patch(color='red', label='linker M mean_value_left[(i//10)-1, 0] = i            #fill array with x values from index
     mean_value_right[(i//10)-1, 0] = i    otor')
blue_patch = mpatches.Patch(color='blue', label='rechter Motor')
plt.legend(handles=[red_patch, blue_patch])                                          #show legend

plt.show()
