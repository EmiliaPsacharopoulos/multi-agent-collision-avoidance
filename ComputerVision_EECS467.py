# and April Tag code from https://github.com/Kazuhito00/AprilTag-Detection-Python-Sample/blob/main/sample.py  

import numpy as np
import cv2
#from pupil_apriltags import Detector
import apriltag
import argparse
import copy
import time
# import serial
# from pathlib import Path
# from xbee import XBee


def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument("--families", type=str, default='tag36h11')
    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)



    args = parser.parse_args()

    return args


def main():
	
	#initialization and open the port
	
	# ser = serial.Serial('/dev/ttyUSB0')
	# ser.baudrate = 9600
	# ser.bytesize = serial.EIGHTBITS #number of bits per bytes
	
	#exp.ser = serial.Serial('COM4', baudrate=9600, bytesize=serial.EIGHTBITS)
	#exp.ser.write(chr(1))
	#ser.parity = serial.PARITY_NONE #set parity check: no parity
	#ser.stopbits = serial.STOPBITS_ONE #number of stop bits
    	#ser.timeout = None          #block read
	#ser.timeout = 5               #non-block read
    	#ser.timeout = 2              #timeout block read
	#ser.xonxoff = False     #disable software flow control
	#ser.rtscts = False     #disable hardware (RTS/CTS) flow control
	#ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
	#print(ser.portstr)
	  
	#print("We are not in the open")
	
	#while (1):
   		#if(ser.in_waiting > 0):
        	#if ser.isOpen():
            		#print("We are in the open")
            		#ser.write(5)
            		
            		#ser.write('Its Working')
           		# read_data = ser.read(10)
          		#  response  = ser.readline()
            		#print("Data received : ")
           		# ser.close()
        	#else:
            		#print ("Can not open serial port")
           		# ser.open()
	
	#get args
	args = get_args()
	cap_device = args.device
	cap_width = args.width
	cap_height = args.height
	families = args.families
	nthreads = args.nthreads
	quad_decimate = args.quad_decimate
	quad_sigma = args.quad_sigma
	refine_edges = args.refine_edges
	decode_sharpening = args.decode_sharpening
	debug = args.debug

	# Capturing video through webcam
	#webcam = cv2.VideoCapture(0)
	i = -1
	webcam = cv2.VideoCapture(i)
	while not webcam.isOpened():
    		webcam = cv2.VideoCapture(i)
    		i = i + 1
	print(i-1)
	
	webcam.set(cv2.CAP_PROP_FRAME_WIDTH, cap_width)
	webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, cap_height)

	# Detector 
	options = apriltag.DetectorOptions(
		families=families,
		nthreads=nthreads,
		quad_decimate=quad_decimate,
		refine_edges=refine_edges,
		debug=debug
	)
	
	at_detector = apriltag.Detector()
	elapsed_time = 0
	
	#every_four = 0;

	#serial_port = serial.Serial('/dev/ttyUSB4', 9600)
	#xbee2 = XBee(serial_port)
	# Start a while loop
	while(1):
		start_time = time.time()

		# Reading the video from the
		# webcam in image frames
		ret, imageFrame = webcam.read()
		if not ret:
			break

		debug_image = copy.deepcopy(imageFrame)

		# Convert the imageFrame in
		# BGR(RGB color space) to
		# HSV(hue-saturation-value)
		# color space
		gray = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
		hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

		#Declare coordinates
		robot1_x = 0
		robot1_y = 0
		robot2_x = 0
		robot2_y = 0
		
		# APRIL TAGS!!!
		tags = at_detector.detect(
			gray
		)
		key = cv2.waitKey(1)
		if key == 27:  # ESC
			break
		
		for r in tags:
			if(r.tag_id == 0):
				#robot #1 coords 
				robot1_x = int(r.center[0])
				robot1_y = int(r.center[1])
			if(r.tag_id == 1):
				#robot #2 coords 
				robot2_x = int(r.center[0])
				robot2_y = int(r.center[1])
		
		debug_image = draw_tags(debug_image, tags, elapsed_time)
		elapsed_time = time.time() - start_time
		cv2.imshow('AprilTag Detect Demo', debug_image)

		# Program Termination
		cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
		if cv2.waitKey(10) & 0xFF == ord('q'):
			cap.release()
			cv2.destroyAllWindows()
			break
			
		# #transmit here!
		# robot1_x_upper, robot1_x_lower = (robot1_x & 0xFFFFFFFF).to_bytes(2, 'big')
		# robot1_y_upper, robot1_y_lower = (robot1_y & 0xFFFFFFFF).to_bytes(2, 'big')
		# robot1_x_final = (robot1_x_lower>>2) | ((robot1_x_upper& 0b11 )<<6)
		# robot1_y_final = (robot1_y_lower>>2) | ((robot1_y_upper& 0b11 )<<6)
		
		# robot2_x_upper, robot2_x_lower = (robot2_x & 0xFFFFFFFF).to_bytes(2, 'big')
		# robot2_y_upper, robot2_y_lower = (robot2_y & 0xFFFFFFFF).to_bytes(2, 'big')
		# robot2_x_final = (robot2_x_lower>>2) | ((robot2_x_upper& 0b11 )<<6)
		# robot2_y_final = (robot2_y_lower>>2) | ((robot2_y_upper& 0b11 )<<6)
		
		# final = [robot1_x_final, robot1_y_final,
		# 		robot2_x_final, robot2_y_final]
			
		# finalTx = bytes(final)
		
		# ser.write(finalTx) #we transmitted serially over GPIO for EECS 373


		#finalTx = str(finalTx)
		
		#print(len(testArr))
		#print(final)
		#xbee2.write(finalTx)
		#response = xbee2.wait_read_frame()
		
		#print(final)
    		
		#print(ser.read())
		#if(every_four == 5):
		#	ser.write(testArr)
		#	every_four = 0
		#else:
		#	every_four = every_four + 1
		
		
	

def draw_tags(
    image,
    tags,
    elapsed_time,
):
    for tag in tags:
        tag_family = tag.tag_family
        tag_id = tag.tag_id
        center = tag.center
        corners = tag.corners

        center = (int(center[0]), int(center[1]))
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

        cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

        cv2.line(image, (corner_01[0], corner_01[1]),
                (corner_02[0], corner_02[1]), (255, 0, 0), 2)
        cv2.line(image, (corner_02[0], corner_02[1]),
                (corner_03[0], corner_03[1]), (255, 0, 0), 2)
        cv2.line(image, (corner_03[0], corner_03[1]),
                (corner_04[0], corner_04[1]), (0, 255, 0), 2)
        cv2.line(image, (corner_04[0], corner_04[1]),
                (corner_01[0], corner_01[1]), (0, 255, 0), 2)

        # cv2.putText(image,
        #            str(tag_family) + ':' + str(tag_id),
        #            (corner_01[0], corner_01[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
        #            0.6, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(image, str(tag_id), (center[0] - 10, center[1] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.putText(image,
               "Elapsed Time:" + '{:.1f}'.format(elapsed_time * 1000) + "ms",
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
               cv2.LINE_AA)

    return image


if __name__ == '__main__':
    main()
