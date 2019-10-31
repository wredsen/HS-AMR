import numpy as np
from matplotlib import pyplot as plt

sample_array = np.arange(1050,dtype=float).reshape((70,15))
gauss_array = np.arange(210,dtype=float).reshape((70,3))    #fist expected value second variance

j=0
l=10
k=0
test_array = np.arange(1050).reshape((70,15))
test_array[0,0] = 3
test_array[1,0] = 4

for i in range(10,80):
   j=0
   k=0
   with open("/home/sebastian/Dokumente/Mitschriften/WISE_2020/Hauptseminar/HS-AMR/Perception/sharp_s1/output_"+str(i)+"cm.txt","r") as file:
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
            
            
            
            

for k in range(0,15):
    print(sample_array[1,k])   



for i in range(0,70):
     gauss_array[i,1] = np.mean(sample_array[i])
     gauss_array[i,2] = np.std(sample_array[i])
     print(gauss_array[i,1])
     print(gauss_array[i,2])


for i in range(0,70):
     gauss_array[i,0] = i+10


plt.plot(gauss_array[:, 0],gauss_array[:, 1],'o')
p = np.polyfit(gauss_array[:, 0],gauss_array[:, 1],6)
print(p)
f = np.poly1d(p) 
plt.plot(gauss_array[:, 0],f(gauss_array[:, 0]), 'b-',label="Polyfit")
plt.show()


