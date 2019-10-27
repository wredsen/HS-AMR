##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed 
import serial

serial_port = '/dev/ttyACM0';
baud_rate = 38400; #In arduino, Serial.begin(baud_rate)
write_to_file_path = "~/Dokumente/Mitschriften/WISE 2020/Hauptseminar/output.txt";

output_file = open(write_to_file_path, "w+");
ser = serial.Serial(serial_port, baud_rate)
while True:
    line = ser.readline();
    line = line.decode("utf-8") #ser.readline returns a binary, convert to string
    print(line);
    output_file.write(line);