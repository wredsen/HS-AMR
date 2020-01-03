##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed 
import serial
import time
i = 10;
timeout = time.time() + 5   # 5 seconds from now

serial_port = '/dev/ttyACM0';
baud_rate = 38400; #In arduino, Serial.begin(baud_rate)

ser = serial.Serial(serial_port, baud_rate)


for i in range(10,81):

    write_to_file_path = "/home/sebastian/Dokumente/Mitschriften/WISE_2020/Hauptseminar/HS-AMR/Perception/Sharp Sensoren/sharp_3_3_dig/output_"+str(i)+"cm.txt";
    output_file = open(write_to_file_path, "w+");


    while True:
      line = ser.readline();
      line = line.decode("utf-8") #ser.readline returns a binary, convert to string
      print(line);
      output_file.write(line);
      if time.time() > timeout:
         break
    
    input("press enter to continue..."+str(i))
    timeout = time.time() + 5   # 5 seconds from now
