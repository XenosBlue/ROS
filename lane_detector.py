#!/usr/bin/env python


import rospy 
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_msg_python.msg import custom

pub = rospy.Publisher('Coefficients',custom,queue_size=10)
On  = False
cv_image = None
frame_no = 0

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
		
	return (left_avg,right_avg)


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




def process_frame(frame):

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
	except:
		avg_lines = ([0,0],[0,0])

	return avg_lines



def image_callback(data):
	try:
		bridge = CvBridge()
		global cv_image
		cv_image = bridge.imgmsg_to_cv2(data,desired_encoding ='passthrough')
	except CvBrigeError as e:
		print(e)



def LD_callback(event):
	msg = custom()
	try:
		global cv_image,pub,frame_no
		l,r = process_frame(cv_image)
		frame_no += 1
		msg.header.seq = frame_no
		msg.header.stamp = rospy.Time.now()
   		msg.header.frame_id = "lanes"
		msg.left_lane = l
		msg.right_lane = r

	except :
		print("publish error")

	rospy.loginfo(msg)
	pub.publish(msg)





			
def start():
	rospy.init_node('node_1',anonymous = True)
	rospy.Subscriber("FRAMES",Image,self.LD_callback)
	timer = rospy.Timer(rospy.Duration(0.5),LD_callback)
	rospy.spin()
	timer.shutdown()



if __name__ == '__main__':
	start()







