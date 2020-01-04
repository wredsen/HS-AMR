import numpy as np
from matplotlib import pyplot as plt
import matplotlib.pylab as pylab
import os

dirname = os.path.dirname(__file__)  # get absulute file path

# array that contains the raw sample values extracted from the 3 diffrent surfaces
sample_surface_array = np.arange(90, dtype=float).reshape((45, 2))
# array that contains the raw sample values extracted from the 4 difrent hights
sample_hight_array = np.arange(150, dtype=float).reshape((75, 2))
# array that contains the raw sample
sample_velocity_array = np.arange(120, dtype=float).reshape((60,2))

# fist value expected value, second value variance
gauss_surface_array = np.arange(12, dtype=float).reshape((6, 2))
# fist value expected value, second value variance
gauss_hight_array = np.arange(20, dtype=float).reshape((10, 2))
# fist value expected value, second value variance
gauss_velocity_array = np.arange(16, dtype=float).reshape((8,2))


# fist value expected value, second value variance
uncertainty_surface_array = np.arange(6, dtype=float)
# fist value expected value, second value variance
uncertainty_hight_array = np.arange(10, dtype=float)
# fist value expected value, second value variance
uncertainty_velocity_array = np.arange(8, dtype=float)



j = 0  # variable to itterate thru file
l = 0  # flag if string Initialisierung ok is found
k = 0  # counter for number of lines

p = True


for i in range(0, 6):

    if(i % 2 == 1):
        direction = 'y'

    if(i % 2 == 0):
        direction = 'x'

    # define relative path
    filename = os.path.join(dirname, '../mouse/sample_for_python/1_'+direction+'_'+str(i//2)+'.txt')
    # open all files for task 1 diffrent surfaceses
    with open(filename, "r") as file:
        if file.mode == 'r':  # check if file is readable

            j = 0

            for j, line in enumerate(file):  # itterate thru file i

                try:  # exeption for convertion from string to float
                    t = abs(float(line))
                    # print(t)
                    sample_surface_array[j+((15*(i//2))), i % 2] = t
                except ValueError:
                    print('not a valid number')

                if(j == 14):
                    break

            print('next file')


for i in range(0, 10):

    if(i % 2 == 1):
        direction = 'y'

    if(i % 2 == 0):
        direction = 'x'

    # define relative path
    filename = os.path.join(dirname, '../mouse/sample_for_python/2_'+direction+'_'+str(i//2)+'.txt')
    # open all files for task 1 diffrent surfaceses
    with open(filename, "r") as file:
        if file.mode == 'r':  # check if file is readable

            j = 0

            for j, line in enumerate(file):  # itterate thru file i
                
                try:  # exeption for convertion from string to float
                    #print(j)
                    #print(j+((15*(i//2))))
                    t = abs(float(line))
                    #print(t)
                    sample_hight_array[j+((15*(i//2))), i % 2] = t
                except ValueError:
                    print('not a valid number')

                if(j == 14):
                    break

            print('next file')


for i in range(0, 8):

    if(i % 2 == 1):
        direction = 'y'

    if(i % 2 == 0):
        direction = 'x'
        
    # define relative path
    filename = os.path.join(dirname, '../mouse/sample_for_python/3_'+direction+'_'+str(i//2)+'.txt')
    # open all files for task 1 diffrent surfaceses
    with open(filename, "r") as file:
        if file.mode == 'r':  # check if file is readable

            j = 0
            
            for j, line in enumerate(file):  # itterate thru file i

                try:  # exeption for convertion from string to float
                    t = abs(float(line))
                    # print(t)
                    sample_velocity_array[j+((15*(i//2))), i % 2] = t

                except ValueError:
                    print('not a valid number')

                if(j == 14):
                    break

            print('next file')


# print array of 70th file for debug purpose
if(True):
    print(sample_surface_array)
    print(' ')
    print(sample_hight_array)
    print(' ')
    print(sample_velocity_array)
    print(' ')


# calculate expected value and variance and print them out
i = 0

while i < 5:

    gauss_surface_array[i, 0] = np.mean(sample_surface_array[((i//2)*15):((i//2)*15)+14, 0])      # mean value from x axis
   
    gauss_surface_array[i+1, 0] = np.mean(sample_surface_array[((i//2)*15):((i//2)*15)+14, 1])    # mean value from y axis

    gauss_surface_array[i, 1] = np.std(sample_surface_array[((i//2)*15):((i//2)*15)+14, 0])       # sigma from x axis
  
    gauss_surface_array[i+1, 1] = np.std(sample_surface_array[((i//2)*15):((i//2)*15)+14, 1])     # sigma frim y axis

    uncertainty_surface_array[i] = (gauss_surface_array[i,0]/300)
    uncertainty_surface_array[i+1] = (gauss_surface_array[i+1,0]/300)

    i += 2

# calculate expected value and variance and print them out

i = 0
while i < 9:
    
    gauss_hight_array[i, 0] = np.mean(sample_hight_array[((i//2)*15):((i//2)*15)+15, 0])            # mean value from x axis
    # mean value from y axis
    gauss_hight_array[i+1, 0] = np.mean(sample_hight_array[((i//2)*15):((i//2)*15)+15, 1])

    gauss_hight_array[i, 1] = np.std(sample_hight_array[((i//2)*15):((i//2)*15)+15, 0])             # sigma from x axis
    # sigma from y axis
    gauss_hight_array[i+1, 1] = np.std(sample_hight_array[((i//2)*15):((i//2)*15)+15, 1])
 
    uncertainty_hight_array[i] = (gauss_hight_array[i,0]/300)
    uncertainty_hight_array[i+1] = (gauss_hight_array[i+1,0]/300)

    i += 2

i = 0
# calculate expected value and variance and print them out
while i < 7:

    gauss_velocity_array[i, 0] = np.mean(sample_velocity_array[((i//2)*15):((i//2)*15)+15, 0])            # mean value from x axis
    gauss_velocity_array[i+1, 0] = np.mean(sample_velocity_array[((i//2)*15):((i//2)*15)+15, 1])         # mean value from y axis

    gauss_velocity_array[i, 1] = np.std(sample_velocity_array[((i//2)*15):((i//2)*15)+15, 0])             # sigma from x axis
    gauss_velocity_array[i+1, 1] = np.std(sample_velocity_array[((i//2)*15):((i//2)*15)+15, 1])           # sigma frim y axis
    
    uncertainty_velocity_array[i] = (gauss_velocity_array[i,0]/300)
    uncertainty_velocity_array[i+1] = (gauss_velocity_array[i+1,0]/300)

    i +=2

print(gauss_velocity_array)



if(p == True):

    print('schwarzer Untergrund')
    print('Mittelwert x '+str(gauss_surface_array[0, 0]))
    print('Mittelwert y '+str(gauss_surface_array[1, 0]))
    print('sMU x '+str(uncertainty_surface_array[0]))
    print('sMU y '+str(uncertainty_hight_array[1]))
    print('sigma x '+str(gauss_surface_array[0, 1]))
    print('sigma y '+str(gauss_surface_array[1, 1]))
    print('')
    print('grau Untergrund')
    print('Mittelwert x '+str(gauss_surface_array[2, 0]))
    print('Mittelwert y '+str(gauss_surface_array[3, 0]))
    print('sMU x '+str(uncertainty_surface_array[2]))
    print('sMU y '+str(uncertainty_hight_array[3]))
    print('sigma x '+str(gauss_surface_array[2, 1]))
    print('sigma y '+str(gauss_surface_array[3, 1]))
    print('')
    print('weiß Untergrund')
    print('Mittelwert x '+str(gauss_surface_array[4, 0]))
    print('Mittelwert y '+str(gauss_surface_array[5, 0]))
    print('sMU x '+str(uncertainty_surface_array[4]))
    print('sMU y '+str(uncertainty_hight_array[5]))
    print('sigma x '+str(gauss_surface_array[4, 1]))
    print('sigma y '+str(gauss_surface_array[5, 1]))
    print('')
    print('Höhe normal mm')
    print('Mittelwert x '+str(gauss_hight_array[0, 0]))
    print('Mittelwert y '+str(gauss_hight_array[1, 0]))
    print('sigma x '+str(gauss_hight_array[0, 1]))
    print('sigma y '+str(gauss_hight_array[1, 1]))
    print('')
    print('Höhe 6,3 mm')
    print('Mittelwert x '+str(gauss_hight_array[2, 0]))
    print('Mittelwert y '+str(gauss_hight_array[3, 0]))
    print('sigma x '+str(gauss_hight_array[2, 1]))
    print('sigma y '+str(gauss_hight_array[3, 1]))
    print('')
    print('Höhe 5,85 mm')
    print('Mittelwert x '+str(gauss_hight_array[4, 0]))
    print('Mittelwert y '+str(gauss_hight_array[5, 0]))
    print('sigma x '+str(gauss_hight_array[4, 1]))
    print('sigma y '+str(gauss_hight_array[5, 1]))
    print('')
    print('Höhe 5,8 mm')
    print('Mittelwert x '+str(gauss_hight_array[6, 0]))
    print('Mittelwert y '+str(gauss_hight_array[7, 0]))
    print('sigma x '+str(gauss_hight_array[6, 1]))
    print('sigma y '+str(gauss_hight_array[7, 1]))
    print('')
    print('Höhe 5,3 mm')
    print('Mittelwert x '+str(gauss_hight_array[8, 0]))
    print('Mittelwert y '+str(gauss_hight_array[9, 0]))
    print('sigma x '+str(gauss_hight_array[8, 1]))
    print('sigma y '+str(gauss_hight_array[9, 1]))
    print('')
    print('Geschwindigkeit 25cm/s')
    print('Mittelwert y '+str(gauss_velocity_array[0,0]))
    print('sigma y '+str(gauss_velocity_array[0, 1]))
    print('')
    print('Geschwindigkeit 12,5cm/s')
    print('Mittelwert y '+str(gauss_velocity_array[1,0]))
    print('sigma y '+str(gauss_velocity_array[1, 1]))
    print('')
    print('Geschwindigkeit 5cm/s')
    print('Mittelwert y '+str(gauss_velocity_array[2,0]))
    print('sigma y '+str(gauss_velocity_array[2, 1]))
    print('')
    print('Geschwindigkeit 2,5cm/s')
    print('Mittelwert y '+str(gauss_velocity_array[3,0]))
    print('sigma y '+str(gauss_velocity_array[3, 1]))
   

params = {'legend.fontsize': 25,
          'figure.figsize': (20, 5),
         'axes.labelsize': 28,
         'axes.titlesize': 28,
         'xtick.labelsize':28,
         'ytick.labelsize':28 }
pylab.rcParams.update(params)

width=0.2
ind= np.arange(3)
fig = plt.figure()
ax = fig.add_subplot(111)

xval = [abs(uncertainty_surface_array[0]-31.5),abs(uncertainty_surface_array[2]-31.5),abs(uncertainty_surface_array[4]-31.5)]
xerr = [gauss_surface_array[0,1]/300,gauss_surface_array[2,1]/300,gauss_surface_array[4,1]/300]

rects1 = ax.bar(ind-width/2,xval,width,yerr=xerr,capsize=10)

yval = [abs(uncertainty_surface_array[1]-31.5),abs(uncertainty_surface_array[3]-31.5),abs(uncertainty_surface_array[5]-31.5)]
yerr = [gauss_surface_array[1,1]/300,gauss_surface_array[3,1]/300,gauss_surface_array[5,1]/300]

rects2 = ax.bar(ind+width/2,yval,width,yerr=yerr,capsize=10)


ax.set_ylabel('Messunsicherheit in [mm]')

ax.set_xticks(ind+width)
ax.set_xticklabels( ('schwarz','grau','weiß'))
ax.legend( (rects1[0], rects2[0]), ('Horizontale systematische MU', 'Vertikale systematische MU'))
plt.axhline(0,color='black')
plt.xlabel('Farbe des Untergrundes')

def autolabel(rects):
    for rect in rects:
        #h = rect.get_height()
        h=27
        #ax.text(rect.get_x()+rect.get_width()/2,h, '%.2f'%float(rect.get_height()),ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)


plt.show()


width=0.2
ind= np.arange(5)
fig = plt.figure()
ax = fig.add_subplot(111)

xval = [abs(uncertainty_hight_array[0]-31.5),abs(uncertainty_hight_array[2]-31.5),abs(uncertainty_hight_array[4]-31.5),abs(uncertainty_hight_array[6]-31.5),abs(uncertainty_hight_array[8]-31.5)]
xerr = [gauss_hight_array[0,1]/300,gauss_hight_array[2,1]/300,gauss_hight_array[4,1]/300,gauss_hight_array[6,1]/300,gauss_hight_array[8,1]/300]

rects1 = ax.bar(ind-width/2,xval,width,yerr=xerr,capsize=10)

yval = [abs(uncertainty_hight_array[1]-31.5),abs(uncertainty_hight_array[3]-31.5),abs(uncertainty_hight_array[5]-31.5),abs(uncertainty_hight_array[7]-31.5),abs(uncertainty_hight_array[9]-31.5)]
yerr = [gauss_hight_array[1,1]/300,gauss_hight_array[3,1]/300,gauss_hight_array[5,1]/300,gauss_hight_array[7,1]/300,gauss_hight_array[9,1]/300]

rects2 = ax.bar(ind+width/2,yval,width,yerr=yerr,capsize=10)


ax.set_ylabel('Messunsicherheit in [mm]')
ax.set_xticks(ind+width)
ax.set_xticklabels( ('6','6,3','5,85','5,8','5,3') )
ax.legend( (rects1[0], rects2[0]), ('Horizontale', 'Vertikale') )
plt.axhline(0,color='black')
plt.xlabel('Distanz in [mm]')

def autolabel(rects):
    for rect in rects:
        #h = rect.get_height()
        h=27
        #ax.text(rect.get_x()+rect.get_width()/2,h, '%.2f'%float(rect.get_height()),ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)


plt.show()


width=0.2
ind= np.arange(4)
fig = plt.figure()
ax = fig.add_subplot(111)

xval = [abs(uncertainty_velocity_array[0]-31.5),abs(uncertainty_velocity_array[2]-31.5),abs(uncertainty_velocity_array[4]-31.5),abs(uncertainty_velocity_array[6]-31.5)]
xerr = [gauss_velocity_array[0,1]/300,gauss_velocity_array[2,1]/300,gauss_velocity_array[4,1]/300,gauss_velocity_array[6,1]/300]

rects1 = ax.bar(ind-width/2,xval,width,yerr=xerr,capsize=10)

yval = [abs(uncertainty_velocity_array[1]-31.5),abs(uncertainty_velocity_array[3]-31.5),abs(uncertainty_velocity_array[5]-31.5),abs(uncertainty_velocity_array[7]-31.5)]
yerr = [gauss_velocity_array[1,1]/300,gauss_velocity_array[3,1]/300,gauss_velocity_array[5,1]/300,gauss_velocity_array[7,1]/300]

rects2 = ax.bar(ind+width/2,yval,width,yerr=yerr,capsize=10)


ax.set_ylabel('Messunsicherheit in [mm]')
ax.set_xticks(ind+width)
ax.set_xticklabels( ('25','12,5','5','2,5') )
ax.legend( (rects1[0], rects2[0]), ('Horizontale', 'Vertikale') )
plt.axhline(0,color='black')
plt.xlabel('Geschwindigkeit in [cm/s]')

def autolabel(rects):
    for rect in rects:
        #h = rect.get_height()
        h=27
        #ax.text(rect.get_x()+rect.get_width()/2,h, '%.2f'%float(rect.get_height()),ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)


plt.show()