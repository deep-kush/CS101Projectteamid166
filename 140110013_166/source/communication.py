from xbee import XBee
import serial
import cv2
import os
import re

hotlist = ["4","5","7"]
numberOfCars = 6 #number can be set to anything as per need
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600) #opening of serial port; do 'dmesg' in terminal to figure out which port xbee is connected to; generally it is /dev/ttyUSB0

while True:
	if numberOfCars == 0:
		ser.write('u')
		break
	resp = ser.read()
	#print resp
	if ord(resp) == 118: #ascii of 'v',stands for verify or 'check' if the car is stolen or not
		#print "connected"
		cap= cv2.VideoCapture(1)  #capture image
		ret, frame= cap.read()
		#img2= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		cv2.imwrite('input.jpg',frame)
		cap.release()
		os.system("python extract.py input.jpg  out") #call image processing module
		os.system("tesseract out.jpg out")	#call tesseract OCR
		f = open("out.txt",'r')	
		numberplate = f.read()
		'''if '4' in numberplate:
			numberplate = '4' ''' 
		match = re.search(r'\d',numberplate) #extract the numeral in the license plate
		numberplate = match.group()
		
		if numberplate not in hotlist:  #number plate not in hotlist
			if numberplate == '0':
				ser.write('a')
			if numberplate == '1':
				ser.write('b')
			if numberplate == '2':
				ser.write('c')
			if numberplate == '3':
				ser.write('d')
			if numberplate == '4':
				ser.write('e')
			if numberplate == '5':
				ser.write('f')
			if numberplate == '6':
				ser.write('g')
			if numberplate == '7':
				ser.write('h')
			if numberplate == '8':
				ser.write('i')
			if numberplate == '9':
				ser.write('j')
		else: #numberplate in hotlist
			if numberplate == '0':
				ser.write('A')
			if numberplate == '1':
				ser.write('B')
			if numberplate == '2':
				ser.write('C')
			if numberplate == '3':
				ser.write('D')
			if numberplate == '4':
				ser.write('E')
			if numberplate == '5':
				ser.write('F')
			if numberplate == '6':
				ser.write('G')
			if numberplate == '7':
				ser.write('H')
			if numberplate == '8':
				ser.write('I')
			if numberplate == '9':
				ser.write('J')							
		numberOfCars -= 1	
ser.close() # close serial port
#end program

