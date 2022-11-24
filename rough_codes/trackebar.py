#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 as cv
import numpy as np

hueLow=10
hueHigh=20
satLow=10
satHigh=250
valLow=10
valHigh=250

def onTrack1(val):
    global hueLow
    hueLow=val
    print('Hue Low',hueLow)
def onTrack2(val):
    global hueHighyellow_detect
    hueHigh=val
    print('Hue High',hueHigh)
def onTrack3(val):
    global satLow
    satLow=val
    print('Sat Low',satLow)
def onTrack4(val):
    global satHigh
    satHigh=val
    print('Sat High',satHigh)
def onTrack5(val):
    global valLow
    valLow=val
    print('Val Low',valLow)
def onTrack6(val):
    global valHigh
    valHigh=val
    print('Val High',valHigh)


width=640
height=360
cv.namedWindow('myTracker')
cv.moveWindow('myTracker',width,0)


cv.createTrackbar('Hue Low','myTracker',10,179,onTrack1)
cv.createTrackbar('Hue High','myTracker',20,179,onTrack2)
cv.createTrackbar('Sat Low','myTracker',10,255,onTrack3)
cv.createTrackbar('Sat High','myTracker',250,255,onTrack4)
cv.createTrackbar('Val Low','myTracker',10,255,onTrack5)
cv.createTrackbar('Val High','myTracker',250,255,onTrack6)


def getContours(binary_image):      
    contours, hierarchy = cv.findContours(binary_image.copy(), 
                                            cv.RETR_EXTERNAL,
                                            cv.CHAIN_APPROX_SIMPLE)
    return contours

def get_contour_center(contour):
    M = cv.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def draw(contours):
    for c in contours:
        # if(cv.contourArea(c)>100):
            # print(cv.contourArea(c))
            # cv.drawContours(img,c,-1, [0, 255, 0], 3)
            
            cx, cy = get_contour_center(c)
            # 
            print(str(cx)+" "+str(cy))
            # return cx,cy
            # pose_estimation(cx,cy)
            # turn_estimation(cx)

yellow_lower=(22,93,0)
yellow_upper=(45,255,255)

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,3] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp = [0,0,78.99]
		# self.Ki = [0,0,0]
		# self.Kd = [0,0,180.13]
		
		
		#setted for throttle
		# self.Kp = [0,0,0]
		# self.Ki = [0,0,0]
		# self.Kd = [0,0,0]

		# self.Kp = [5,5.22,22]
		# self.Ki = [0,0,0.01]
		# self.Kd = [209.7,209.7,410]

		# self.Kp = [60,60,633]
		# self.Ki = [0,0,0]
		# self.Kd = [450,450,1616]


		# self.Kp = [32.76,32.76,33.76]
		# self.Ki = [0,0,0]
		# self.Kd = [897.3,897.3,563.4]



		# self.Kp = [32.76,32.76,33.76]
		# self.Ki = [0,0,0]
		# self.Kd = [897.3,897.3,568.7]


		self.Kp = [32.76,32.76,33.76]
		self.Ki = [0,0,0]
		self.Kd = [897.3,897.3,568.7]


# possible value
		# self.Kp = [49.92,49.92,99.72]
		# self.Ki = [0,0.0001,0.008]
		# self.Kd = [1500,1500,1500]


		# self.Kp = [32.76,32.76,33.76]
		# self.Ki = [0,0,0]
		# self.Kd = [897.3,897.3,600]

		#-----------------------Add other required variables for pid here ----------------------------------------------
        #   throttle variables
		# self.throttle_error=0
		# self.throttle_prev_error=0
		# self.throttle_sum_error=0

		# self.min_throttle=1000
		# self.max_throttle=2000
        #---------------------------------------------------------------------------------------------------------------------------
			#Array declaration
		self.Error = [0,0,0]
		self.PrevError = [0,0,0]
		self.sumError=[0,0,0]

		self.Min=1000
		self.Max =2000

		



		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error=rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error=rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error=rospy.Publisher('/roll_error', Float64, queue_size=1)







		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z



		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------





		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------


















	#----------------------------------------------------------------------------------------------------------------------


	# def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
#-------------------------------------------------------------------------------------------------------------------
	# 	self.throttle_error= -(self.setpoint[2]-self.drone_position[2])
	# 	# print("error"+str(self.throttle_error))
	# 	print("kp"+str(self.Kp[2]))
	# 	print("Kd"+str(self.Kd[2]))
	# 	self.cmd.rcThrottle=int(1500+ self.throttle_error * self.Kp[2]+ (self.throttle_error-self.throttle_prev_error) * self.Kd[2]+self.throttle_sum_error * self.Ki[2])
		
	# 	if self.cmd.rcThrottle>self.max_throttle:
	# 		self.cmd.rcThrottle=self.max_throttle
	# 	if self.cmd.rcThrottle<self.min_throttle:
	# 		self.cmd.rcThrottle=self.min_throttle
	# 	print("throttle"+str(self.cmd.rcThrottle))


	# 	self.throttle_prev_error=self.throttle_error                               #for updating
	# 	self.throttle_sum_error=self.throttle_sum_error+self.throttle_error






	# #------------------------------------------------------------------------------------------------------------------------


		
	# 	self.command_pub.publish(self.cmd)
	# 	self.throttle_error_pub.publish(self.throttle_error)
#--------------------------------------------------------------------------
         #throttle

	def pid(self):
		self.PrevError[2] = self.Error[2]
		self.sumError[2] += self.Error[2]

		self.PrevError[0] = self.Error[0]
		self.sumError[0] += self.Error[0]

		self.PrevError[1] = self.Error[1]
		self.sumError[1] += self.Error[1]
	
		self.Error[0] = (self.setpoint[0]-self.drone_position[0])
		self.Error[2] = -(self.setpoint[2]-self.drone_position[2])
		self.Error[1] = -(self.setpoint[1]-self.drone_position[1])
        
        


		# if(self.Error[2]<=0.2):
		# 	self.cmd.rcThrottle = int(1500+self.Error[2]*self.Kp[2]+(self.Error[2]-self.PrevError[2])*self.Kd[2]+self.sumError[2]*self.Ki[2])
		# else:
		# 	self.cmd.rcThrottle = int(1500+self.Error[2]*self.Kp[2]+(self.Error[2]-self.PrevError[2])*self.Kd[2])
			
		# print(self.Error[2])
		self.cmd.rcThrottle = int(1500+self.Error[2]*self.Kp[2]+(self.Error[2]-self.PrevError[2])*self.Kd[2]+self.sumError[2]*self.Ki[2])	
		self.cmd.rcRoll = int(1500+self.Error[0]*self.Kp[0]+(self.Error[0]-self.PrevError[0])*self.Kd[0]+self.sumError[0]*self.Ki[0])
		self.cmd.rcPitch = int(1500+self.Error[1]*self.Kp[1]+(self.Error[1]-self.PrevError[1])*self.Kd[1]+self.sumError[1]*self.Ki[1])	
		# print("Kp"+str(self.Kp[2]))
		# print("Ki"+str(self.Ki[2]))
		# print("Kd"+str(self.Kd[2]))

		# print("ROll Kp"+str(self.Kp[0]))
		# print("Roll Kd"+str(self.Kd[0]))

		# print("Pitch Kp"+str(self.Kp[1]))
		# print("Pitch Kd"+str(self.Kd[1]))


		if(self.cmd.rcThrottle > self.Max):
			self.cmd.rcThrottle = self.Max
		if(self.cmd.rcThrottle < self.Min):
			self.cmd.rcThrottle = self.Min

		if(self.cmd.rcRoll > self.Max):
			self.cmd.rcRoll = self.Max
		if(self.cmd.rcRoll < self.Min):
			self.cmd.rcRoll = self.Min


		if(self.cmd.rcPitch > self.Max):
			self.cmd.rcPitch = self.Max
		if(self.cmd.rcPitch < self.Min):
			self.cmd.rcPitch = self.Min



		self.command_pub.publish(self.cmd)




		
		
        # print(self.cmd.rcThrottle)
		
		
		
		# self.command_pub.publish(self.cmd)
		self.alt_error.publish(self.Error[2])


        # Roll Error
		
		
		
		
		# print("Kp"+str(self.Kp[2]))
		# print("Kd" +str(self.Kd[2]))
		
		self.roll_error.publish(self.Error[0])



        # Pitch Error
		
		
		# print("pitch"+str())
		
		
		
		# self.command_pub.publish(self.cmd)
		self.pitch_error.publish(self.Error[1])



def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
#   rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data,"bgr8")

#   br = CvBridge.imgmsg_to_cv2(data, "bgr8")
#   current_frame=br


  hsv=cv.cvtColor(current_frame,cv.COLOR_BGR2HSV)
  blurred = cv.GaussianBlur(hsv, (5, 5), 0)
  mask=cv.inRange(blurred,yellow_lower,yellow_upper)


#   lowerBound=np.array([hueLow,satLow,valLow])
#   upperBound=np.array([hueHigh,satHigh,valHigh])
#   myMask=cv.inRange(hsv,lowerBound,upperBound)

  contours=getContours(mask)
  
  
  cv.imshow("mask",mask)


   
  # Display image
  cv.imshow("camera", current_frame)


   
  cv.waitKey(1)
  draw(contours)

def receive_image():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
#   rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
#   rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, callback)
 
  rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, callback)
  # spin() simply keeps python from exiting until this node is stopped
#   rospy.spin()
 
  # Close down the video stream when done
#   cv.destroyAllWindows()




if __name__ == '__main__':


	 

	e_drone = Edrone()
	r = rospy.Rate(30)   # 0.020 sspecify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
         e_drone.pid()
         receive_image()
         r.sleep()






