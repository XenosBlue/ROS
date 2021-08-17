import os
import re
import cv2
import numpy as np
import matplotlib.pyplot as plt


def gamma_correction(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def lines_average(frame, lines):
	left  = []
	right = []
	for each in lines:
		x1,y1,x2,y2 = each.reshape(4)
		coefficients = np.polyfit((x1,x2),(y1,y2),1)
		m = coefficients[0]
		b = coefficients[1]
		t= 0.315
		if m<-t:
			left.append((m, b))
		else:
			if m>t:
				right.append((m,b))
	left_avg = np.average(left,axis=0)
	right_avg = np.average(right,axis=0)
	left_line  = cvtLine(frame, left_avg)
	right_line = cvtLine(frame, right_avg)
	print("l : ",left_avg)
	print("r : ",right_avg)
	res = np.array([[left_line],[right_line]])
	print("RES : ",res)
	return res


def cvtLine(frame, coefficients):
	m,b = coefficients
	y1 = frame.shape[0]
	y2 = int(y1*(4/5))
	x1 = int((y1-b)/m)
	x2 = int((y2-b)/m)
	return np.array([x1,y1,x2,y2])


def draw_lane(frame, lines):
	frame = np.copy(frame)
	line_filter = np.zeros((frame.shape[0],frame.shape[1], 3), dtype=np.uint8)

	try:
		
		for each in lines:
			for x1,y1,x2,y2 in each:
				cv2.line(line_filter,(x1,y1),(x2,y2),(0,255,0),thickness=6)
		frame = cv2.addWeighted(frame, 0.8,line_filter, 1, 1)
	except:
		print("Null")

	return frame


def focus(frame, points):
	mask = np.zeros_like(frame)
	match_mask_color = 255
	cv2.fillPoly(mask, points, match_mask_color)
	masked_image = cv2.bitwise_and(frame, mask)
	return masked_image




def process_frame(frame,prev):

	try:
		height = frame.shape[0]
		width  = frame.shape[1]

		hc = 1/4 #3/5
		wc = 1/4


		#vertices = [(0,height), (width*wc, height*hc), (width*(1-wc),height*hc), (width,height)]
		vertices = [(0,height), (width/2, height/2),(width,height)]
		grayScale = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
		#darker    = cv2.cvtColor(frame, cv2.COLOR_RGB2HLS)
		darker = gamma_correction(grayScale,0.5)
		blur      = cv2.GaussianBlur(darker, (7,7),0)
		#blur = grayScale
		cannyEdge = cv2.Canny(blur,100,200)
		focused = focus(cannyEdge,np.array([vertices], np.int32))


		lines = cv2.HoughLinesP(focused,rho=6,theta=np.pi/180,threshold=150,lines=np.array([]),minLineLength=50,maxLineGap=25)
		avg_lines  = lines_average(frame,lines)
		final = draw_lane(frame, avg_lines)
		#print("lines : ",lines)
	except:
		try:
			final = draw_lane(frame, prev)
			avg_lines = prev
		except:
			final = frame
			avg_lines = prev
		
		#print("Fin")

	return (final,avg_lines)



video = cv2.VideoCapture('/XXXXXXX/harder_challenge_video.mp4')
prev=0
while(video.isOpened()):
	ret, frame = video.read()
	frame,prev = process_frame(frame,prev)
	try:
		cv2.imshow('Lane',frame)
	except:
		continue
	if cv2.waitKey(10) == ord('q'):
		break
print("EX")
video.release()
cv2.destroyAllWindows()
