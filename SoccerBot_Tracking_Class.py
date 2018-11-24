import time
import numpy as np
import cv2
import sys
import math

class TrackingSoccerBotsFns:
	def __init__(self):
		self.template = cv2.imread('TemplateBW.png',0)
		
		self.SB_Center = np.array([0, 0])
		self.Prev_SB_C = np.array([0, 0])
		self.Displacement = np.array([0,0])
		
		self.SB_Angle = 0
		self.Prev_SB_Angle = 0
	
	def Constrain_Search_Region_Boundaries(self, center, size):
		# simply creates a search box around a center point which is within the 640,480 frame
		Constrained_center = np.clip(center, [size/2, size/2], [639 - size/2, 479 - size/2])	
		xmin = int(Constrained_center[0] - size/2)
		xmax = int(Constrained_center[0] + size/2)
		ymin = int(Constrained_center[1] - size/2)
		ymax = int(Constrained_center[1] + size/2)
		 
		return xmin, xmax, ymin, ymax
		
	def Find_Angle_and_Distance(self, pt1, pt2):
		diff = pt2 - pt1
		angle = (math.atan2(diff[1], diff[0]) * 180 / 3.142) + 180	
		distance = math.sqrt(diff[0]**2 + diff[1]**2)
		return int(angle), int(distance)
		
	def Constrain_Angle(self, angle):
		return(angle % 360)	
		
	def Detect_SB_Tag_Positions(self, Greyscale):
		# Perform the template matching algorithm
		MT_Result = cv2.matchTemplate(Greyscale,self.template,cv2.TM_SQDIFF_NORMED)

		# Now find the positions of the 2 minima: (later 4)
		_, _, min_loc, _ = cv2.minMaxLoc(MT_Result)
		tag1 = np.array(min_loc)	
		MT_Result = cv2.circle(MT_Result, min_loc, 6, (255,255,255), -1)
		
		_, _, min_loc, _ = cv2.minMaxLoc(MT_Result)
		tag2 = np.array(min_loc)		
		MT_Result = cv2.circle(MT_Result, min_loc, 6, (255,255,255), -1)
		
		# Now find the centre of these 2 points:
		Center_P = ((tag1 + tag2) / 2) + [5,5]
		
		return Center_P, tag1, tag2	
		
	def Detect_SB(self, frame):
		GreyScale = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)	
		self.SB_Center, tag1, tag2 = self.Detect_SB_Tag_Positions(GreyScale)
		
		self.SB_Angle, _ 	= self.Find_Angle_and_Distance(tag1, tag2)	
		self.SB_Angle 		= self.Constrain_Angle(self.SB_Angle + 45)
		
		return self.SB_Center, self.SB_Angle
		
	def Track_SB(self, frame):
		# Calculate displacement between current frame and previous frame, and thus predict the best area to do the next search in.
		self.Displacement = self.SB_Center - self.Prev_SB_C 												
		self.Displacement = np.clip(self.Displacement, -10, 10)
		Next_Search_Center = self.SB_Center + self.Displacement	
		
		xmin, xmax, ymin, ymax = self.Constrain_Search_Region_Boundaries(Next_Search_Center, 60)
		Search_Frame = frame[ymin:ymax,xmin:xmax]
			
		position_in_SF, self.SB_Angle = self.Detect_SB(Search_Frame)
		self.SB_Center = position_in_SF + [xmin,ymin]				
			
		# Ensure angle doesn't flip 180 degrees. Flip it back if this has happened
		if(abs(self.SB_Angle - self.Prev_SB_Angle) > 150 and abs(self.SB_Angle - self.Prev_SB_Angle) < 210):
			self.SB_Angle = self.Constrain_Angle(self.SB_Angle + 180)
			
		self.Prev_SB_C = self.SB_Center
		self.Prev_SB_Angle = self.SB_Angle
		
		return self.SB_Center, self.SB_Angle	
		
		
# when to update previous 		
		
# See if we've get our expected colour blobs in our square otherwise perform an object detection process
#Narrowed_Search_Frame_Colour = frame[ymin:ymax,xmin:xmax]
#GausBlur = cv2.GaussianBlur(Narrowed_Search_Frame_Colour, (5, 5), 0)
#hsv = cv2.cvtColor(GausBlur, cv2.COLOR_BGR2HSV)    
#lower_thres = np.array([34,56,23])                        
#upper_thres = np.array([83,255,255])
#maskG = cv2.inRange(hsv, lower_thres, upper_thres)		
		
		
		
		
		
		
		
		
		