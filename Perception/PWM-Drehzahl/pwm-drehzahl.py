import numpy as np
from matplotlib import pyplot as plt

# erster Wert linker Encoder, zweiter Wert rechter Encoder
mean_value_left = np.arange(20,dtype=float).reshape((10,2)) # pro Zeile (PWM, Drehzahl)
mean_value_right = np.arange(20,dtype=float).reshape((10,2)) # pro Zeile (PWM, Drehzahl)

i = 10
while i <= 100:
     print((i/10)-1)
     mean_value_left[int((i/10)-1), 0] = i
     mean_value_right[int((i/10)-1), 0] = i

     with open("/home/sebastian/Dokumente/Mitschriften/WISE_2020/Hauptseminar/HS-AMR/Control/PWM-Drehzahl/LogDaten/NXTData"+str(i)+".txt","r") as file:
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
                              mean_value_right[int((i/10)-1), 1] += right_meas

                              try: 
                                   m = k+1
                                   while line[1][-m] != ";":
                                        m+=1
                                   left_meas = float(line[1][-(m-1):-(k+1)])
                                   mean_value_left[int((i/10)-1), 1] += left_meas
                              except IndexError:
                                   print('m greater array')

                         except ValueError:
                              print('not a valid number')

               mean_value_right[int((i/10)-1), 1] = mean_value_right[int((i/10)-1), 1] / number_of_measures
               mean_value_left[int((i/10)-1), 1] = mean_value_left[int((i/10)-1), 1] / number_of_measures
               
     i = i+10

print(mean_value_right[:,0])
print(mean_value_left[:,1])
print(mean_value_right[:,1])
            
          

plt.plot(mean_value_left[:,0], mean_value_left[:,1],'o')
plt.plot(mean_value_right[:,0], mean_value_right[:,1],'o')
fit_left = np.polyfit(mean_value_left[:, 0],mean_value_left[:, 1],1)
fit_right = np.polyfit(mean_value_right[:, 0],mean_value_right[:, 1],1)
print(fit_left)
print(fit_right)
func_left = np.poly1d(fit_left) 
func_right = np.poly1d(fit_right) 
plt.plot(mean_value_left[:, 0],func_left(mean_value_left[:, 0]), 'b-',label="Fit-Left")
plt.plot(mean_value_right[:, 0],func_right(mean_value_right[:, 0]), 'b-',label="Fit-Right")
plt.show()
