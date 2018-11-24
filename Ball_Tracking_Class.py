import numpy as np
import time
import cv2

class TrackingBallFns:
	def __init__(self):
		self.Ball_Center = np.array([0,0])
		self.Prev_Ball_C = np.array([0,0])
		self.Displacement = np.array([0,0])
		
		self.Gross_Travel_Displacement = np.array([0,0])
		self.Position_200ms_ago = np.array([0,0])
		self.Last_Pos_Update_TS = 0
		
	def Constrain_Search_Region_Boundaries(self, center, size):
		# simply creates a search box around a center point which is within the 640,480 frame
		Constrained_center = np.clip(center, [size/2, size/2], [639 - size/2, 479 - size/2])	
		xmin = int(Constrained_center[0] - size/2)
		xmax = int(Constrained_center[0] + size/2)
		ymin = int(Constrained_center[1] - size/2)
		ymax = int(Constrained_center[1] + size/2)
		 
		return xmin, xmax, ymin, ymax	
	
	def Detect_Ball(self, frame):
		# Blurs frame, does an HSV mask, finds largest blob and return the center of it
		GausBlur = cv2.GaussianBlur(frame, (5, 5), 0)
		hsv = cv2.cvtColor(GausBlur, cv2.COLOR_BGR2HSV)    
		lower_thres = np.array([7,170,25])                        
		upper_thres = np.array([12,255,255])
		mask = cv2.inRange(hsv, lower_thres, upper_thres)	

		_,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		if len(contours) != 0:
			#find the biggest area
			c = max(contours, key = cv2.contourArea)

			M = cv2.moments(c)
			
			if(M['m00'] == 0):
				return np.array([0,0]), 0
			
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			
			self.Ball_Center = np.array([cx,cy])
			
			return self.Ball_Center, 1
		return np.array([0,0]), 0
		
	def Track_Ball(self, frame):
		# Identifies the displacement of the ball from 1 frame to next and searches in small region around this predicted point
		self.Displacement = self.Ball_Center - self.Prev_Ball_C
		self.Displacement = np.clip(self.Displacement, -20, 20)
		Next_Search_Center = self.Ball_Center + self.Displacement
		
		xmin, xmax, ymin, ymax = self.Constrain_Search_Region_Boundaries(Next_Search_Center, 60)
		Search_Frame = frame[ymin:ymax,xmin:xmax]
		
		position_in_SF, Found_Flag = self.Detect_Ball(Search_Frame)
		self.Ball_Center = position_in_SF + [xmin,ymin]
		
		self.Prev_Ball_C = self.Ball_Center
	
		return self.Ball_Center, Found_Flag
	
	def perp(self, a):
		b = np.empty_like(a)
		b[0] = -a[1]
		b[1] = a[0]
		return b

	def Segment_Intersect(self, a1,a2, b1,b2):
		da = a2-a1
		db = b2-b1
		dp = a1-b1
		dap = self.perp(da)
		denom = np.dot(dap, db)
		num = np.dot(dap, dp )
		return (num / denom.astype(float))*db + b1
		
	def Ball_Heading_For_Goal(self, a1,a2, b1,b2):
		Intersect_Point = self.Segment_Intersect(a1,a2, b1,b2)
		
		if(Intersect_Point[1] > 0 and Intersect_Point[1] < 479):
			Intersect_Point[1] = np.clip(Intersect_Point[1], 120, 330)
			return Intersect_Point, True	
		return np.array([50,225]), False
		
		
	def Predict_Ball_Path(self, frame_timestamp):
		if(time.time() > self.Last_Pos_Update_TS + 0.2):
			self.Last_Pos_Update_TS = time.time()
			self.Gross_Travel_Displacement = self.Ball_Center - self.Position_200ms_ago
			self.Position_200ms_ago = self.Ball_Center
		
		Expected_Position_in_200ms = self.Ball_Center + self.Gross_Travel_Displacement
		Expected_Position_in_2s = self.Ball_Center + (self.Gross_Travel_Displacement * 10)
		
		return Expected_Position_in_2s
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		

		
		
		
		
		
		
		
		