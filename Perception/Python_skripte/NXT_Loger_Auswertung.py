import numpy as np                      
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import os

dirname = os.path.dirname(__file__)               #get absulute file path
# erster Wert linker Encoder, zweiter Wert rechter Encoder
sample_value_left  = np.arange(500,dtype=float)
sample_value_right = np.arange(500,dtype=float)

mean_value_left = np.arange(12,dtype=float).reshape((6,2)) # pro Zeile (PWM, Drehzahl)
mean_value_right = np.arange(12,dtype=float).reshape((6,2)) # pro Zeile (PWM, Drehzahl)


for i in range(0,6):  
     filename = os.path.join(dirname,'../LightsensorSample/surface_'+str(i)+'.txt')  #define relative path
     sampcount_r = 0
     sampcount_l = 0
     with open(filename,"r") as file:                                                #open file from relative path
          if file.mode == 'r':                                                       #check if file is open as "read"
               number_of_measures = 0                                                #counter for sampeled values
               for line in enumerate(file):                                          #itterarate thru line of file 
                    if line[1].count(";") > 0:                                       #count if ";" is in the tupel line line[0] is the line number
                         if(sampcount_r < 500):
                              try:                                                   #exception for stirng to float conversion 
                                                                                     #add one sample to the array (counter)

                                   k = 1
                                   while line[1][-k] != ";":                         #start itterate backwards to search for first ";"
                                       k+=1
                                   
                                   right_meas = float(line[1][-(k-1):])              #cast from string to float line from -(k-1) to end of line
                                   sample_value_right[sampcount_r] = right_meas                 #save sample to array 
                                   sampcount_r += 1

                              except ValueError:            
                                   print('not a valid number')

                         if(sampcount_l < 500):
                              try:                                                   #exception for stirng to float conversion 
                                   m = k+1

                                   while line[1][-m] != ";":                         #start itterate backwards to search for first ";" beginning from k+1
                                          m+=1

                                   left_meas = float(line[1][-(m-1):-(k)])           #cast from string to float line from -(m-1) to -(k-1) second sample 
                                   sample_value_left[sampcount_l] = left_meas                   #save sample to array
                                   sampcount_l += 1

                              except ValueError:
                                   print('not a valid number')
                         

                         if (sampcount_r > 500 and sampcount_l > 500):
                              break



               #print(sample_value_left)
               mean_value_right[i, 0] = np.mean(sample_value_right)         #calculate the mean of the sampeled values
               mean_value_left[i, 0]  = np.mean(sample_value_left)
               mean_value_right[i, 1] = np.std(sample_value_right)
               mean_value_left[i, 1]  = np.std(sample_value_left)
               print()
               
     

print('black bright mean R  '+str(mean_value_right[0,0]))
print('black bright mean L  '+str(mean_value_left[0,0]))
print('black bright sigma R '+str(mean_value_right[0,1]))
print('black bright sigma L '+str(mean_value_left[0,1]))
print(' ')
print('black dark mean    R '+str(mean_value_right[1,0]))
print('black dark mean    L '+str(mean_value_left[1,0]))
print('black dark sigma   R '+str(mean_value_right[1,1]))
print('black dark sigma   L '+str(mean_value_left[1,1]))
print(' ')
print('gray bright mean   R '+str(mean_value_right[2,0]))
print('gray bright mean   L '+str(mean_value_left[2,0]))
print('gray bright sigma  R '+str(mean_value_right[2,1]))
print('gray bright sigma  L '+str(mean_value_left[2,1]))
print(' ')
print('gray dark mean     R '+str(mean_value_right[3,0]))
print('gray dark mean     L '+str(mean_value_left[3,0]))
print('gray dark sigma    R '+str(mean_value_right[3,1]))
print('gray dark sigma    L '+str(mean_value_left[3,1]))
print(' ')
print('white bright mean  R '+str(mean_value_right[4,0]))
print('white bright mean  L '+str(mean_value_left[4,0]))
print('white bright sigma R '+str(mean_value_right[4,1]))
print('white bright sigma L '+str(mean_value_left[4,1]))
print(' ')
print('white dark mean    R '+str(mean_value_right[5,0]))
print('white dark mean    L '+str(mean_value_left[5,0]))
print('white dark sigma   R '+str(mean_value_right[5,1]))
print('white dark sigma   L '+str(mean_value_left[5,1]))


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
red_patch = mpatches.Patch(color='red', label='linker Motor')
blue_patch = mpatches.Patch(color='blue', label='rechter Motor')
plt.legend(handles=[red_patch, blue_patch])                                          #show legend

plt.show()
