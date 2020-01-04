import numpy as np                      
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.pylab as pylab
import os

dirname = os.path.dirname(__file__)               #get absulute file path
# erster Wert linker Encoder, zweiter Wert rechter Encoder
sample_value_left  = np.arange(200,dtype=float)
sample_value_right = np.arange(200,dtype=float)

mean_value_left = np.arange(12,dtype=float).reshape((6,2)) # pro Zeile (PWM, Drehzahl)
mean_value_right = np.arange(12,dtype=float).reshape((6,2)) # pro Zeile (PWM, Drehzahl)


for i in range(0,6):  
     filename = os.path.join(dirname,'../LightsensorSample/sample richtige Kalibrierung//s_'+str(i)+'.txt')  #define relative path
     sampcount_r = 0
     sampcount_l = 0
     with open(filename,"r") as file:                                                #open file from relative path
          if file.mode == 'r':                                                       #check if file is open as "read"
               number_of_measures = 0                                                #counter for sampeled values
               for line in enumerate(file):                                          #itterarate thru line of file 
                    if line[1].count(";") > 0:                                       #count if ";" is in the tupel line line[0] is the line number
                         if(sampcount_r < 200):
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

                         if(sampcount_l < 200):
                              try:                                                   #exception for stirng to float conversion 
                                   m = k+1

                                   while line[1][-m] != ";":                         #start itterate backwards to search for first ";" beginning from k+1
                                          m+=1

                                   left_meas = float(line[1][-(m-1):-(k)])           #cast from string to float line from -(m-1) to -(k-1) second sample 
                                   sample_value_left[sampcount_l] = left_meas                   #save sample to array
                                   sampcount_l += 1

                              except ValueError:
                                   print('not a valid number')
                         

                         if (sampcount_r > 200 and sampcount_l > 200):
                              break



               #print(sample_value_left)
               mean_value_right[i, 0] = np.mean(sample_value_right)         #calculate the mean of the sampeled values
               mean_value_left[i, 0]  = np.mean(sample_value_left)
               mean_value_right[i, 1] = np.std(sample_value_right)
               mean_value_left[i, 1]  = np.std(sample_value_left)
               
               
     

print('schwarz hell Mittelwert   R '+str(mean_value_right[0,0]))
print('schwarz hell Mittelwert   L '+str(mean_value_left[0,0]))
print('schwarz hell sigma        R '+str(mean_value_right[0,1]))
print('schwarz hell sigma        L '+str(mean_value_left[0,1]))
print(' ')
print('schwarz dunkel Mittelwert R '+str(mean_value_right[1,0]))
print('schwarz dunkel Mittelwert L '+str(mean_value_left[1,0]))
print('schwarz dunkel sigma      R '+str(mean_value_right[1,1]))
print('schwarz dunkel sigma      L '+str(mean_value_left[1,1]))
print(' ')
print('grau hell Mittelwert      R '+str(mean_value_right[2,0]))
print('grau hell Mittelwert      L '+str(mean_value_left[2,0]))
print('grau hell sigma           R '+str(mean_value_right[2,1]))
print('grau hell sigma           L '+str(mean_value_left[2,1]))
print(' ')
print('grau dunkel Mittelwert    R '+str(mean_value_right[3,0]))
print('grau dunkel Mittelwert    L '+str(mean_value_left[3,0]))
print('grau dunkel sigma         R '+str(mean_value_right[3,1]))
print('grau dunkel sigma         L '+str(mean_value_left[3,1]))
print(' ')
print('weiß hell Mittelwert      R '+str(mean_value_right[4,0]))
print('weiß hell Mittelwert      L '+str(mean_value_left[4,0]))
print('weiß hell sigma           R '+str(mean_value_right[4,1]))
print('weiß hell sigma           L '+str(mean_value_left[4,1]))
print(' ')
print('weiß dunkel Mittelwert    R '+str(mean_value_right[5,0]))
print('weiß dunkel Mittelwert    L '+str(mean_value_left[5,0]))
print('weiß dunkel sigma         R '+str(mean_value_right[5,1]))
print('weiß dunkel sigma         L '+str(mean_value_left[5,1]))



params = {'legend.fontsize': 25,
          'figure.figsize': (20, 5),
         'axes.labelsize': 28,
         'axes.titlesize': 28,
         'xtick.labelsize':20,
         'ytick.labelsize':28 }
pylab.rcParams.update(params)


width=0.2
ind= np.arange(6)
fig = plt.figure()
ax = fig.add_subplot(111)


yvals=[mean_value_left[0,0],mean_value_left[1,0],mean_value_left[2,0],mean_value_left[3,0],mean_value_left[4,0],mean_value_left[5,0]]
rects1 = ax.bar(ind,yvals,width)
zvals=[mean_value_right[0,0],mean_value_right[1,0],mean_value_right[2,0],mean_value_right[3,0],mean_value_right[4,0],mean_value_right[5,0]]
rects2 = ax.bar(ind+width,zvals,width)

ax.set_ylabel('Helligkeit in [%]')
ax.set_xticks(ind+width)
ax.set_xticklabels( ('schwarz hell','schwarz dunkel', 'grau hell','grau dunkel','weiß hell', 'weiß dunkel') )
ax.legend( (rects1[0], rects2[0]), ('linker Sensor', 'rechter Sensor') )

def autolabel(rects):
    for rect in rects:
        h = rect.get_height()
        ax.text(rect.get_x()+rect.get_width()/2.,h-0.2, '%d'%int(h),
                ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)


width=0.2
ind= np.arange(6)
fig = plt.figure()
ax = fig.add_subplot(111)


yvals=[mean_value_left[0,1],mean_value_left[1,1],mean_value_left[2,1],mean_value_left[3,1],mean_value_left[4,1],mean_value_left[5,1]]
rects1 = ax.bar(ind,yvals,width)
zvals=[mean_value_right[0,1],mean_value_right[1,1],mean_value_right[2,1],mean_value_right[3,1],mean_value_right[4,1],mean_value_right[5,1]]
rects2 = ax.bar(ind+width,zvals,width)

ax.set_ylabel('Messunsicherheit in [%]')
ax.set_xticks(ind+width)
ax.set_xticklabels( ('schwarz hell','schwarz dunkel', 'grau hell','grau dunkel','weiß hell', 'weiß dunkel') )
ax.legend( (rects1[0], rects2[0]), ('linker Sensor', 'rechter Sensor') )

def autolabel(rects):
    for rect in rects:
        h = rect.get_height()
        ax.text(rect.get_x()+rect.get_width()/2.,h, '%.2f'%float(h),
                ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)



plt.show()
