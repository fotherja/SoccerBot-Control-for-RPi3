# Code to control our SoccerBots

import Serial_Stream_Class
import Frame_Grabber_Class
import Ball_Tracking_Class
import SoccerBot_Tracking_Class
import time
import numpy as np
import cv2
import sys
import math

Goal_1A = np.array([50, 120])
Goal_1B = np.array([50, 330])

ss = Serial_Stream_Class.SerialStream().start()
fg = Frame_Grabber_Class.PiVideoStream((640,480), 40).start() 
bt = Ball_Tracking_Class.TrackingBallFns() 
st = SoccerBot_Tracking_Class.TrackingSoccerBotsFns()          
time.sleep(1.0)
	
def Draw_Line_With_Angle(Initial_Point, Line_angle, Line_length):
	PointX = int(Initial_Point[0] + Line_length * math.cos(Line_angle * 3.142 / 180.0));
	PointY = int(Initial_Point[1] + Line_length * math.sin(Line_angle * 3.142 / 180.0));

	Final_Point = np.array([PointX, PointY])
	return Final_Point
	
def GoTo_Position(Bot_Angle, Bot_Position, Target_Position):				# Calculates the angle and distance from us to the target location
	Angle_Bot_to_Target, Distance_Bot_to_Target = st.Find_Angle_and_Distance(Bot_Position, Target_Position)
	Error_Angle = st.Constrain_Angle(Bot_Angle - Angle_Bot_to_Target + 360)
	
	if(Error_Angle > 270):	
		LR_Differential_Speed = int(max(-10, (Error_Angle - 360)/9))	
		LR_Common_Speed = int(-min(6, Distance_Bot_to_Target/5))
	elif(Error_Angle < 90):
		LR_Differential_Speed = int(min(10, Error_Angle/9))						
		LR_Common_Speed = int(-min(6, Distance_Bot_to_Target/5))
	elif(Error_Angle > 180):
		LR_Differential_Speed = int(min(10, Error_Angle/9))
		LR_Common_Speed = int(min(6, Distance_Bot_to_Target/5))
	else:
		LR_Differential_Speed = int(max(-10, (Error_Angle - 360)/9))
		LR_Common_Speed = int(min(6, Distance_Bot_to_Target/5))	
	
	print(Error_Angle)

	if (Distance_Bot_to_Target < 5):
		Tx_data = (128, 128)
		return(Tx_data)
	else:
		Tx_data = (128 + LR_Common_Speed + LR_Differential_Speed, 128 + LR_Common_Speed - LR_Differential_Speed)
		return(Tx_data)
	
#----------------------------------Main-------------------------------------------
# Do an initial search for the SoccerBot and ball over the whole arena
frame, Last_Frame_Num, frame_timestamp	= fg.read() 
SB_Center, SB_Angle 					= st.Detect_SB(frame)
Ball_Center, _ 							= bt.Detect_Ball(frame)

while(True):	
	# Block until we have a new frame
	frame_num = fg.Frame_Number()
	while(Last_Frame_Num == frame_num):
		time.sleep(0.001)
		frame_num = fg.Frame_Number()
	frame, Last_Frame_Num, frame_timestamp = fg.read()
	
	# Track objects	and update their position states
	Ball_Center, _ 			= bt.Detect_Ball(frame)
	SB_Center, SB_Angle 	= st.Track_SB(frame)
	
	# Work out where the ball will be in the future:
	Expected_Position = bt.Predict_Ball_Path(frame_timestamp)
		
	# Is the ball heading towards our goal and going to be there in < 2 seconds?
	Goal_Target, Yes_No_Flag = bt.Ball_Heading_For_Goal(Goal_1A, Goal_1B, Ball_Center, Expected_Position)
			
	# Now we know the positions of everything we move the SoccerBot to the intersect point to block the goal:
	Tx_data = GoTo_Position(SB_Angle, SB_Center, Goal_Target)	
	ss.write(Tx_data)	

		
		
		
		
	# Debug
	#print(SB_Angle)
	frame = cv2.circle(frame, (int(Goal_1A[0]),int(Goal_1A[1])), 4, (0,255,0), -1)	
	frame = cv2.circle(frame, (int(Goal_1B[0]),int(Goal_1B[1])), 4, (0,255,0), -1)
	
	frame = cv2.circle(frame, (int(Ball_Center[0]),int(Ball_Center[1])), 4, (0,0,255), -1)
	frame = cv2.circle(frame, (int(SB_Center[0]),int(SB_Center[1])), 4, (0,0,255), -1)
	frame = cv2.circle(frame, (int(Expected_Position[0]),int(Expected_Position[1])), 4, (0,255,255), -1)
	frame = cv2.circle(frame, (int(Goal_Target[0]),int(Goal_Target[1])), 6, (255,255,0), -1)
	
	Heading_P = Draw_Line_With_Angle(SB_Center, ((SB_Angle + 180) % 360), 25)
	frame = cv2.line(frame, (int(SB_Center[0]),int(SB_Center[1])), (int(Heading_P[0]),int(Heading_P[1])), (0,0,255), 2)
	cv2.imshow("Frames", frame) 
	
	c = cv2.waitKey(5)	
	if 'q' == chr(c & 255):
		break
	
cv2.destroyAllWindows()
fg.stop()
ss.stop()









