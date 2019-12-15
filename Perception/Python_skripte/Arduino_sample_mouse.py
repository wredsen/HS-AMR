##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed 
import serial
import time
import os

dirname = os.path.dirname(__file__)               #get absulute file path
filename = os.path.join(dirname,'../mouse.txt')       #define relative path

serial_port = '/dev/ttyACM0';
baud_rate = 38400; #In arduino, Serial.begin(baud_rate)

ser = serial.Serial(serial_port, baud_rate)



for i in range(0,16):

    write_to_file_path = filename
    output_file = open(write_to_file_path, "w+");


    #input("press enter to sample..."+str(i))
    ser.write(4)
    line = ser.readline();
    line = line.decode("utf-8") #ser.readline returns a binary, convert to string
    print(line);
    output_file.write(line);
    
    
    input("press enter to reset..."+str(i))
    #ser.write(6)
    timeout = time.time() + 5   # 5 seconds from now
