import sys, os, serial, threading

def monitor():
	ser = serial.Serial(COMPORT,BAUDRATE,timeout=0);
	while(1):
		line = ser.readline();
		if(line != ""):
			#print line[:-1]         # strip \n
			fields = line[:-1].split('; ');
			ID = fields[0]
            TIME = int(fields[1])
			print("device ID: ", ID);
			print(ID);
			#write to file
           	text_file = open("Pdata.log", "w")
           	line = str(TIME) + ": " + str(CT) + "\n"
           	text_file.write(line)
           	text_file.close()

    print("Stop Monitoring");


import sys, os, serial, threading

def monitor():

   ser = serial.Serial(COMPORT, BAUDRATE, timeout=0)

   while (1):
       line = ser.readline()
       if (line != ""):
           #print line[:-1]         # strip \n
           fields = line[:-1].split('; ');

           // ID = fields[0]
                 // TIME = int(fields[1])
           # print fields
           print "device ID: ", ID
           # write to file
           text_file = open("Pdata.log", "w")
           line = str(TIME) + ": " + str(CT) + "\n"
           text_file.write(line)
           text_file.close()

       # do some other things here

   print "Stop Monitoring"

""" -------------------------------------------
MAIN APPLICATION
"""  
# screen -L /dev/tty.usbserial-A10015y4
#Anything coming in the serial port will automatically get written toa file called screen.0, in the current working directory. To stop sending, type control-C.
#https://www.tigoe.com/pcomp/code/OSX/10/
print "Start Serial Monitor"
print

COMPORT = 4;
BAUDRATE = 115200

# https://forum.arduino.cc/index.php?topic=6464.0 bottom
monitor()