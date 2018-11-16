# Code to find the position and orientation of our SoccerBots. Runs quickly at 25Hz ie 40ms
#
#
# To do:
# - Attempt to go to a location GoTo(x,y)
# - Allow for identifying and tracking 2 SoccerBots   -> Find 4 points and associate the 2 points closest to our colour tag
# - Allow for identifying and tracking the ball
#

import Serial_Stream_Class
import Frame_Grabber_Class
import time
import numpy as np
import cv2
import sys
import math

template = cv2.imread('TemplateBW.png',0)
ss = Serial_Stream_Class.SerialStream().start()
vs = Frame_Grabber_Class.PiVideoStream((640,480), 40).start()            
time.sleep(1.0)

def Find_Points(img):
	# Perform the template matching algorithm
	res = cv2.matchTemplate(img,template,cv2.TM_SQDIFF_NORMED)

	# Now find the positions of the 2 minima:
	_, _, min_loc, _ = cv2.minMaxLoc(res)
	pt1 = np.array(min_loc)	
	res = cv2.circle(res, min_loc, 6, (255,255,255), -1)
	
	_, _, min_loc, _ = cv2.minMaxLoc(res)
	pt2 = np.array(min_loc)		
	res = cv2.circle(res, min_loc, 6, (255,255,255), -1)
	
	# Now find the centre of these 2 points:
	Center_P = ((pt1 + pt2) / 2) + [5,5]
	
	return Center_P, pt1, pt2
	
def Find_Angle(pt1, pt2):
	diff = pt2 - pt1
	angle = (math.atan2(diff[1], diff[0]) * 180 / 3.142) + 180
	return int(angle)
	
def Constrain_Angle(angle):
	return(angle % 360)

def Find_Distance(pt1, pt2):
	distance = math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)	
	return int(distance)
	
def Draw_Line_With_Angle(Initial_Point, Line_angle, Line_length):
	PointX = int(Initial_Point[0] + Line_length * math.cos(Line_angle * 3.142 / 180.0));
	PointY = int(Initial_Point[1] + Line_length * math.sin(Line_angle * 3.142 / 180.0));

	Final_Point = np.array([PointX, PointY])
	return Final_Point
	
def Constrain_Search_Region_Boundaries(Next_Area_To_Search):
	if(Next_Area_To_Search[0] < 50):
		xmin = 0
		xmax = 100
	elif(Next_Area_To_Search[0] >= 589):
		xmin = 539
		xmax = 639
	else:
		xmin = int(Next_Area_To_Search[0] - 50)
		xmax = int(Next_Area_To_Search[0] + 50)	
	
	if(Next_Area_To_Search[1] < 50):
		ymin = 0
		ymax = 100
	elif(Next_Area_To_Search[1] >= 429):
		ymin = 379
		ymax = 479
	else:
		ymin = int(Next_Area_To_Search[1] - 50)
		ymax = int(Next_Area_To_Search[1] + 50)	
		
	return xmin, xmax, ymin, ymax
	
def GoTo_Position(Bot_Angle, Bot_Position, Target_Position):				# Calculates the angle and distance from us to the target location
	Angle_Bot_to_Target = Find_Angle(Bot_Position, Target_Position)
	Distance_Bot_to_Target = Find_Distance(Bot_Position, Target_Position)
	
	Error_Angle = Constrain_Angle(Bot_Angle - Angle_Bot_to_Target + 360)
	print(Error_Angle)
	
	if(Error_Angle > 180):	
		LR_Differential_Speed = int(max(-8, (Error_Angle - 360)/10))			#-2
	else:
		LR_Differential_Speed = int(min(8, Error_Angle/10))						#2
		
	LR_Common_Speed = 10
	
	if (Distance_Bot_to_Target < 40):
		Tx_data = (128, 128)
		return(Tx_data, 1)
	else:
		#Tx_data = (128, 128)
		Tx_data = (128 + LR_Common_Speed + LR_Differential_Speed, 128 + LR_Common_Speed - LR_Differential_Speed)
		return(Tx_data, 0)
	
	
#----------------------------------Main-------------------------------------------
# Do an initial search for the SoccerBot over the whole arena
frame, frame_num = vs.read() 
GreyScale = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)	
Center_P, pt1, pt2 = Find_Points(GreyScale)

Angle = Find_Angle(pt1, pt2)
Previous_Center_P = Center_P

Target_A = np.array([100, 100])
Target_B = np.array([400, 400])
Target_Position = Target_A
		
while(True):	
	# Calculate displacement between current frame and previous frame, and thus predict the best area to do the next search in.
	Displacement = Center_P - Previous_Center_P													# Could set an upper bound on this
	Previous_Center_P = Center_P
	Previous_Angle = Angle
	Next_Area_To_Search = Center_P + Displacement		
	
	# Get a new frame
	frame, frame_num = vs.read() 
	GreyScale = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)		
	
	# Get a subregion of the whole frame around our location of interest
	xmin, xmax, ymin, ymax = Constrain_Search_Region_Boundaries(Next_Area_To_Search)
	Narrowed_Search_Frame = GreyScale[ymin:ymax,xmin:xmax]	
	
	# Find new centre point and new angle.
	Center_P, pt1, pt2 = Find_Points(Narrowed_Search_Frame)
	Center_P = Center_P + [xmin,ymin]
	Angle = Constrain_Angle(Find_Angle(pt1, pt2) + 45)
	
	#Ensure angle doesn't flip 180 degrees. Flip it back if this has happened
	if(abs(Angle - Previous_Angle) > 150 and abs(Angle - Previous_Angle) < 210):
		Angle += 180
		
	#If the direction we're trying to move in is opposite to our heading vector flip it
	if(Find_Distance((0,0), Displacement) > 5):			
		Travel_Angle = Find_Angle((0,0), Displacement)
		
		if(abs(Angle - Travel_Angle) > 150 and abs(Angle - Travel_Angle) < 210):
			Angle += 180

	Angle = Constrain_Angle(Angle)		
		
	# Control SoccerBot	
	Tx_data, Arrived_Flag = GoTo_Position(Angle, Center_P, Target_Position)	
	ss.write(Tx_data)

	if(Arrived_Flag == 1 and Need_To_Leave_Flag == 0):
		Need_To_Leave_Flag = 1
		
		if(np.all(Target_Position == Target_A)):
			Target_Position = Target_B
		else:
			Target_Position = Target_A
	
	if(Arrived_Flag == 0):
		Need_To_Leave_Flag = 0
		
		
	# Debug
	#print(Angle)
	frame = cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (255,0,0))
	
	frame = cv2.circle(frame, (int(Target_A[0]),int(Target_A[1])), 4, (0,255,0), -1)	
	frame = cv2.circle(frame, (int(Target_B[0]),int(Target_B[1])), 4, (0,255,0), -1)
	
	Heading_P = Draw_Line_With_Angle(Center_P, Angle, 25)
	frame = cv2.line(frame, (int(Center_P[0]),int(Center_P[1])), (int(Heading_P[0]),int(Heading_P[1])), (0,0,255), 2)
	cv2.imshow("Frames", frame) 
	
	c = cv2.waitKey(100)	
	if 'q' == chr(c & 255):
		break

	
cv2.destroyAllWindows()
vs.stop()
ss.stop()





