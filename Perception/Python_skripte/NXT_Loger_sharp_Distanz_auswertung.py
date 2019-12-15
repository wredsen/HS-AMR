import numpy as np                      
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import os

dirname = os.path.dirname(__file__)               #get absulute file path
# erster Wert linker Encoder, zweiter Wert rechter Encoder
sample_value_distance = np.arange(200,dtype=float)


mean_value = np.arange(16,dtype=float).reshape((8,2)) # pro Zeile (PWM, Drehzahl)


l=0

for i in range(0,8):  
               filename = os.path.join(dirname,'../Sharp Sensoren/Distanz sample/Sensor_'+str(l+1)+'/s'+str(l+1)+'_'+str((i+1)*10)+'.txt')  #define relative path
               sampcount = 0 

               with open(filename,"r") as file:                                                #open file from relative path
                    if file.mode == 'r':                                                       #check if file is open as "read"
                         number_of_measures = 0                                                #counter for sampeled values
                         for line in enumerate(file):                                          #itterarate thru line of file 
                              if line[1].count(";") > 0:                                       #count if ";" is in the tupel line line[0] is the line number
                                   if(sampcount < 200):
                                        try:                                                   #exception for stirng to float conversion 
                                                                                               #add one sample to the array (counter)

                                             k = 1
                                             while line[1][-k] != ";":                         #start itterate backwards to search for first ";"
                                                  k+= 1
                                             
                                             meas = float(line[1][-(k-1):])                    #cast from string to float line from -(k-1) to end of line
                                             sample_value_distance[sampcount] = meas           #save sample to array 
                                             sampcount += 1

                                        except ValueError:            
                                             print('not a valid number')

                                   if (sampcount > 200):
                                        break



                         #print(sample_value_left)
                         mean_value[i, 0] = np.mean(sample_value_distance)           #calculate the mean of the sampeled values
                         #print(mean_value[i, 0])
                         mean_value[i, 1] = np.std(sample_value_distance)            #calculate the sigma of the sampeled values

          
print(mean_value[:,1])     


#print('Mittelwert='+str(full_array[1]))
#print('Mittelwert='+str(full_array[1]))
#print('sigma = '+str(full_array))


width=0.2
ind= np.arange(8)
fig = plt.figure()
ax = fig.add_subplot(111)


wvals=[10.04,18.54,27.39,39.57,47.52,49.65,49.47,45.935]
werr=[1.37054734,0.78638413,5.13494817,6.00875195,6.94835232,6.03717649,6.64748825,5.77847514]
rects1 = ax.bar(ind-width*2,wvals,width,yerr=werr,capsize=15)
xvals=[11.255,18.86,29.125,41.525,52.365,60.61,63.055,64.11]
rects2 = ax.bar(ind-width,xvals,width)
yvals=[10.505,19.705,32.92,45.895,58.1,66.685,67.055,70.385]
rects3 = ax.bar(ind,yvals,width)
zvals=[10.195,18.875,29.1,38.67,46.035,49.25,51.165,49.275]
rects4 = ax.bar(ind+width,zvals,width)


ax.set_ylabel('gemessene Distanz in [cm]')
ax.set_xticks(ind+width)
ax.set_xticklabels( ('10 cm','20 cm', '30 cm','40 cm','50 cm', '60 cm', '70 cm','80 cm') )
ax.legend( (rects1[0], rects2[0],rects3[0],rects4[0]), ('Sensor 1', 'Sensor 2','Sensor 3','Sensor 4') )

def autolabel(rects):
    for rect in rects:
        h = rect.get_height()
        ax.text(rect.get_x()+rect.get_width()/2.,h-0.2, '%d'%int(h),
                ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)
autolabel(rects3)
autolabel(rects4)

plt.show()