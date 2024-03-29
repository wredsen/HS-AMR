import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
from pynverse import inversefunc
import os

dirname = os.path.dirname(__file__)

sample_array = np.arange(1065,dtype=float).reshape((71,15)) #array that contains the raw sample values extracted from .txt 
gauss_array = np.arange(213,dtype=float).reshape((71,3))    #fist distace 10-80cm, second value expected value, third value variance

j=0            #variable to itterate thru file
l=0            #flag if string Initialisierung ok is found
k=0            #counter for number of lines


for i in range(10,81): #itterate thru all files
   j=0
   k=0
   l=0
   filename = os.path.join(dirname, '../Sharp Sensoren/Messreihe_Sharp_S1_good/output_'+str(i)+'cm.txt')
   with open(filename,"r") as file: #open i'ten file 
        if file.mode == 'r': #check if file is readable
           for j, line in enumerate(file): #itterate thru file i 

               if (k < 15 and l == 1):     #if flag is set and number of lines < 15 true 
          
                    try:                   #exeption for convertion from string to float
                         t = float(file.readline())
                         sample_array[i-10,k] = t
                         k+=1   
                    except ValueError:
                         print('not a valid number')


               if line =='Initialisierung ok\n': #loking for string to start convertion
                    print('gefunden')
                    l = 1 
                  
               j+=1

               if k > 15:    
                    print('next file')
                    print(i)  
                    break
            
            
            
            

for k in range(0,15):         #print array of 70th file for debug purpose
    print(sample_array[70,k])  

print(' ') 


 
for i in range(0,71):         #calculate expected value and variance and print them out
     gauss_array[i,1] = np.mean(sample_array[i]) #mean value = expected value
     gauss_array[i,2] = np.std(sample_array[i])  #variance
     print('Erwartungswert     '+str(i+10)+'  '+str(gauss_array[i,1]))
     print('Standardabweichung '+str(i+10)+'  '+str(gauss_array[i,2]))


for i in range(0,71):         #generate x values 10-80
     gauss_array[i,0] = i+10


#plot voltage(distance)
plt.figure()
plt.errorbar(gauss_array[:, 0],gauss_array[:, 1],yerr= gauss_array[:, 2],fmt='o')  #plot dots with error bars 
p = np.polyfit(gauss_array[:, 0],gauss_array[:, 1],5)                              #make a polynomial fit of 6 degree
print(p)                                                                           #print coifficions
f = np.poly1d(p)                                                                   #build function with coifficions
plt.plot(gauss_array[:, 0],f(gauss_array[:, 0]), '-',label="Polyfit")             #plot polyfit function
plt.xlabel("Distance in cm")
plt.ylabel("Spannung in V")
red_patch = mpatches.Patch(color='orange', label='fit')
plt.legend(handles=[red_patch])

#plot Distance(voltage)
plt.figure()
u = np.polyfit(gauss_array[:,1],gauss_array[:,0],5)
print(u)
b = np.poly1d(u)
x1 = np.arange(0.1, 1.22, 0.02)
plt.plot(x1,b(x1),'-',color='orange')
plt.ylabel("Distance in cm")
plt.xlabel("Spannung in V")
red_patch = mpatches.Patch(color='orange', label='inv fit')
plt.legend(handles=[red_patch])

plt.show()                                                                         #show plots


 