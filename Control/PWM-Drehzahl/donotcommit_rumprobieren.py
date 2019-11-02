import numpy as np
from matplotlib import pyplot as plt

sample_array = np.arange(10,dtype=float).reshape((2,5))

j=0
l=10
k=0

for i in range(10,80):
   j=0
   k=0
   with open("/home/wredi/Desktop/HS-AMR/Perception/sharp_s1/output_"+str(i)+"cm.txt","r") as file:
        if file.mode == 'r':
           for j, line in enumerate(file): 
               if (k < 15 and j > 30):
          
                  try:
                         t = float(file.readline())
                         sample_array[i-10,k] = t
                         k+=1   
                  except ValueError:
                         print('not a valid number')
                  
                  
                    
               j+=1

               if k > 15:   
                    print('next file')
                    print(i)  
                    break
            
            

