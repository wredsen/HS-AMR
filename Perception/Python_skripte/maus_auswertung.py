import numpy as np
from matplotlib import pyplot as plt
import os

dirname = os.path.dirname(__file__)               #get absulute file path

sample_array = np.arange(45,dtype=float).reshape((15,3)) #array that contains the raw sample values extracted from .txt 
gauss_array = np.arange(213,dtype=float).reshape((71,3))    #fist value ppi, second value expected value, third value variance

j=0            #variable to itterate thru file
l=0            #flag if string Initialisierung ok is found
k=0            #counter for number of lines


filename = os.path.join(dirname,'../PWM-Drehzahl/mit_last/pwm_'+str(i)+'.txt')  #define relative path


with open("/home/sebastian/Dokumente/Mitschriften/WISE_2020/Hauptseminar/HS-AMR/Perception/mouse/output_X.txt","r") as file: #open i'ten file 
        if file.mode == 'r': #check if file is readable
          for j, line in enumerate(file): #itterate thru file i 
          
            try:                   #exeption for convertion from string to float
              t = float(line)
              print(t)
              sample_array[j,1] = t
            except ValueError:
              print('not a valid number')

          print('next file')
            

with open("/home/sebastian/Dokumente/Mitschriften/WISE_2020/Hauptseminar/HS-AMR/Perception/mouse/output_Y.txt","r") as file: #open i'ten file 
        if file.mode == 'r': #check if file is readable
          for j, line in enumerate(file): #itterate thru file i 
          
            try:                   #exeption for convertion from string to float
              t = float(line)
              print(t)
              sample_array[j,2] = t
            except ValueError:
              print('not a valid number')

          print('finish')
            
            
            
            
#print array of 70th file for debug purpose
print(sample_array[:,1])
print(sample_array[:,2])  
print(' ') 


#calculate expected value and variance and print them out
mean_x = np.mean(sample_array[:,1]) #mean value = expected value
mean_y = np.mean(sample_array[:,2])  #variance
print('Mittelwert x    '+str(mean_x))
print('Mittelwert y '+str(mean_y))

print('pixel_per_mm_x= '+str(mean_x/250)) #250 mm test distance
print('pixel_per_mm_y= '+str(mean_y/250)) #250 mm test distance

 