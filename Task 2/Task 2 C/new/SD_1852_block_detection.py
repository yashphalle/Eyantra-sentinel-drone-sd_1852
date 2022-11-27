#!/usr/bin/env python3

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv
import numpy as np

i=0
center_x=-1
center_y=-1
detected = False

cam_width=640
cam_height=480

#points_array = [[0, 0, 10], [2, 0, 10], [2, 2, 10], [-2, 2,10], [-2, -2, 10], [2, -2, 10], [2, 0, 10], [0, 0, 10]]
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
    global detected
    for c in contours:
        if cv.contourArea(c)>500:
            global center_x,center_y
            center_x,center_y= get_contour_center(c)
            # print(cv.contourArea(c))
            detected=True
        #     print(detected)
        # print("Centers Are"+str(center_x)+" "+str(center_y))
           

yellow_lower=(22,93,0)
yellow_upper=(45,255,255)

class Edrone():
    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        
        self.drone_position = [0, 0, 0]
        self.setted_x = 0
        self.setted_y = 0
       
        self.setpoint = [0,0,0]
        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [32.76, 32.76, 33.76]
        self.Ki = [0, 0, 0]
        self.Kd = [897.3, 897.3, 568.4]
        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.Error = [0, 0, 0]
        self.PrevError = [0, 0, 0]
        self.sumError = [0, 0, 0]
        self.Min = 1000
        self.Max = 2000

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        #													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        #																	You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher(
            '/drone_command', edrone_msgs, queue_size=1)
        self.alt_error = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.pitch_error = rospy.Publisher(
            '/pitch_error', Float64, queue_size=1)
        self.roll_error = rospy.Publisher(
            '/roll_error', Float64, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',
                         PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------

        # ------------------------------------------------------------------------------------------------------------

        self.arm()
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
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude

    def altitude_set_pid(self, alt):
        # This is just for an example. You can change the ratio/fraction value accordingly
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.08
        self.Kd[2] = alt.Kd * 0.3

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3


    def go_down(self):
        print("go down called")
        e_drone.setpoint[0] = self.drone_position[0]
        e_drone.setpoint[1] = self.drone_position[1]
        e_drone.setpoint[2] = self.drone_position[2]-10
        while self.drone_position[2] <= 20:
            e_drone.setpoint[2]=self.drone_position[2]+1
            e_drone.pid()
            r.sleep()

        
        


    def go_left(self):
        
        print("go left called")
        
        while (center_x<=(cam_width/2-60)):
            print("while of go to left")
            e_drone.setpoint = self.drone_position
            e_drone.setpoint[2] = 20
            e_drone.setpoint[0]=self.drone_position[0]+1

            while not rospy.is_shutdown():
                e_drone.pid()
                r.sleep()
                if -0.2<e_drone.Error[0]<0.2 :
                    e_drone.setted_x=self.drone_position[0]
                    break
                else:
                    while not rospy.is_shutdown():
                        e_drone.pid()
                        r.sleep()
            
            

    def go_right(self):
        print("go right called")
        e_drone.setpoint[2] = 20
        
        
        while (center_x>=(cam_width/2+60)):
            e_drone.setpoint = self.drone_position
            e_drone.setpoint[0]=self.drone_position[0]-1
            e_drone.setpoint[2] = 20
            while not rospy.is_shutdown():
                e_drone.pid()
                r.sleep()
                if -0.2<e_drone.Error[0]<0.2 :
                    e_drone.setted_x=self.drone_position[0]
                    break
                else:
                    while not rospy.is_shutdown():
                        e_drone.pid()
                        r.sleep()

    
    def go_top(self):
        print("go top called")
        e_drone.setpoint= self.drone_position
        
        while(center_y<=(cam_height/2-120)):
            e_drone.setpoint[1]=self.drone_position[1]-3
            while not rospy.is_shutdown():
                e_drone.pid()
                r.sleep()
                if -0.2<e_drone.Error[0]<0.2 :
                    e_drone.setted_y=self.drone_position[1]
                    break
                else:
                    while not rospy.is_shutdown():
                        e_drone.pid()
                        r.sleep()
        e_drone.setted_y=self.drone_position[1]-0.5

    def go_bottom(self):
        
        print("go bottom called")
        print(e_drone.setted_x)
        while(center_y>(cam_height/2+120)):
            print("while of bottom")
            print(center_y)
            e_drone.setpoint[1]=self.drone_position[1]+3
            # e_drone.setpoint[2] = 20
            # e_drone.setpoint[0]=e_drone.setted_x
            while not rospy.is_shutdown():
                # e_drone.setpoint[2] = 20
                # # e_drone.setpoint[1]=self.drone_position[1]-1
                # e_drone.setpoint[0]=e_drone.setted_x
                e_drone.pid()
                r.sleep()
                if -0.2<e_drone.Error[1]<0.2 :
                    e_drone.setted_y=self.drone_position[1]
                    print(e_drone.setted_y)
                    break
                else:
                    while not rospy.is_shutdown():
                        e_drone.pid()
                        r.sleep()
        e_drone.setted_y=self.drone_position[1]+0.5

    def pid(self):
        self.Error[0] = (self.setpoint[0]-self.drone_position[0])
        self.Error[1] = -(self.setpoint[1]-self.drone_position[1])
        self.Error[2] = -(self.setpoint[2]-self.drone_position[2])

        self.cmd.rcRoll = int(1500+self.Error[0]*self.Kp[0]+(self.Error[0]-self.PrevError[0])*self.Kd[0]+self.sumError[0]*self.Ki[0])

        if(self.cmd.rcRoll > self.Max):
            self.cmd.rcRoll = self.Max
        if(self.cmd.rcRoll < self.Min):
            self.cmd.rcRoll = self.Min
        self.PrevError[0] = self.Error[0]
        if(self.Error[0] < 0.2 and self.Error[0] > -0.2):
            self.sumError[0] += self.Error[0]
        else:
            self.sumError[0] = 0
        self.roll_error.publish(self.Error[0])

        # Pitch Error
        self.Error[1] = -(self.setpoint[1]-self.drone_position[1])
        self.cmd.rcPitch = int(1500+self.Error[1]*self.Kp[1]+(self.Error[1]-self.PrevError[1])*self.Kd[1]+self.sumError[1]*self.Ki[1])
        if(self.cmd.rcPitch > self.Max):
            self.cmd.rcPitch = self.Max
        if(self.cmd.rcPitch < self.Min):
            self.cmd.rcPitch = self.Min
        self.PrevError[1] = self.Error[1]
        if(self.Error[1] < 0.2 and self.Error[1] > -0.2):
            self.sumError[1] += self.Error[1]
        else:
            self.sumError[1] = 0
        self.pitch_error.publish(self.Error[1])

        # Throttle Error
        self.Error[2] = -(self.setpoint[2]-self.drone_position[2])
        self.cmd.rcThrottle = int(1500+self.Error[2]*self.Kp[2]+(self.Error[2]-self.PrevError[2])*self.Kd[2]+self.sumError[2]*self.Ki[2])
        if(self.cmd.rcThrottle > self.Max):
            self.cmd.rcThrottle = self.Max
        if(self.cmd.rcThrottle < self.Min):
            self.cmd.rcThrottle = self.Min
        self.PrevError[2] = self.Error[2]
        if(self.Error[2] > -0.5 and self.Error[2] < 0.5):
            self.sumError[2] += self.Error[2]
        else:
            self.sumError[2] = 0
        self.alt_error.publish(self.Error[2])
        
        
        
        self.command_pub.publish(self.cmd)
        
def callback(data):
    # print("captured..")
    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(data,"bgr8")
    hsv=cv.cvtColor(current_frame,cv.COLOR_BGR2HSV)
    blurred = cv.GaussianBlur(hsv, (5, 5), 0)
    mask=cv.inRange(blurred,yellow_lower,yellow_upper)
    contours=getContours(mask)
    draw(contours)
    cv.imshow("mask",mask)
    cv.imshow("camera", current_frame)
    cv.waitKey(1)


def receive_image():
    rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, callback)
   


def move():
    receive_image()
    e_drone.pid()
    r = rospy.Rate(30)
    # points_array = [[0, 0, 23], [2, 0, 23], [2, 2, 23], [-2, 2,23], [-2, -2, 23], [2, -2, 23], [2, 0, 23], [0, 0, 23]]
    points_array = [[0, 0, 22], [7, 0, 22], [7, 6, 22], [-7, 6,22], [-7, -6, 22], [7, -6, 22], [7, 0, 22], [0, 0, 22]]
    # i = 0
    # i = 0
    global detected
    global i
    while i != 8 and detected==False:
        e_drone.setpoint = points_array[i]
        while True:
            e_drone.pid()
            if(e_drone.drone_position[0] < points_array[i][0]+0.2 and e_drone.drone_position[0] > points_array[i][0]-0.2
               and e_drone.drone_position[1] < points_array[i][1]+0.2 and e_drone.drone_position[1] > points_array[i][1]-0.2 and
               e_drone.drone_position[2] < points_array[i][2]+0.2 and e_drone.drone_position[2] > points_array[i][2] - 0.2):
                #print(e_drone.drone_position)
                # receive_image()
                break
            r.sleep()

        i += 1
    print("loop breaked...")
    print(i)
    e_drone.setpoint = points_array[i-1]
    print(e_drone.setpoint)
    # while e_drone.drone_position!=points_array[i]:
    #     e_drone.setpoint = points_array[i]
    # while True:
    #     e_drone.pid()
    #     # if(e_drone.drone_position[0] < points_array[i][0]+0.2 and e_drone.drone_position[0] > points_array[i][0]-0.2
    #     #        and e_drone.drone_position[1] < points_array[i][1]+0.2 and e_drone.drone_position[1] > points_array[i][1]-0.2 and
    #     #        e_drone.drone_position[2] < points_array[i][2]+0.2 and e_drone.drone_position[2] > points_array[i][2] - 0.2):
    #     #         #print(e_drone.drone_position)
    #     #         # receive_image()
    #     #         break
    #     r.sleep()


def go_to_goal():
    global center_y,center_x
    global cam_width,cam_height
    print("go to goal called")
    # e_drone.go_down()

    # while not(center_x<(cam_width/2)-20 and center_x>(cam_width/2)+20 and center_y<(cam_height/2)-0 and center_y>(cam_height/2)+20):
    if(center_x<(cam_width/2) and center_y<(cam_height/2)):
               print("second_quadrant")
               e_drone.go_left()
               e_drone.go_top()
            

    elif(center_x<(cam_width/2) and center_y>(cam_height/2)):
               print("third Quadrant")
               e_drone.go_left()
               e_drone.go_bottom()

               

    elif(center_x>cam_width/2 and center_y<cam_height/2):
               print("first Quadrant")
               e_drone.go_right()
               e_drone.go_top()


    elif(center_x>cam_width/2 and center_y>cam_height/2):
               print("fourth quadrant")
               e_drone.go_right()
               e_drone.go_bottom()

            
    # print("loop break hua")        
    e_drone.setpoint= [e_drone.setted_x,e_drone.setted_y,20]
    print("final setpoint")
    print(e_drone.setpoint)  
    while not rospy.is_shutdown():
        print("pid loop")
        e_drone.pid()
        r.sleep()
        if -0.2<e_drone.Error[0]<0.2 and -0.2<e_drone.Error[1]<0.2 and -0.2<e_drone.Error[2]<0.2 :
            break
        else:
            while not rospy.is_shutdown():
                e_drone.pid()
                r.sleep()  
    

   
        





        
if __name__ == '__main__':
    # global i
    e_drone = Edrone()
    t = time.time()
    r = rospy.Rate(30)
    while time.time() - t < 5:
        pass
    
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    # while not rospy.is_shutdown():
    move()
    
    # points_array = [[0, 0, 20], [2, 0, 20], [2, 2, 20], [-2, 2,20], [-2, -2, 20], [2, -2, 20], [2, 0, 20], [0, 0, 20]]
    points_array = [[0, 0, 22], [7, 0, 22], [7, 6, 22], [-7, 6,22], [-7, -6, 22], [7, -6, 22], [7, 0, 22], [0, 0, 22]]
    # i = 0
    # while (not(e_drone.drone_position[0] < points_array[i-1][0]+0.2 and e_drone.drone_position[0] > points_array[i-1][0]-0.2
    #            and e_drone.drone_position[1] < points_array[i-1][1]+0.2 and e_drone.drone_position[1] > points_array[i-1][1]-0.2 and
    #            e_drone.drone_position[2] < points_array[i-1][2]+0.2 and e_drone.drone_position[2] > points_array[i-1][2] - 0.2)):
    while not rospy.is_shutdown():
        e_drone.setpoint = points_array[i-1]
        e_drone.pid()
        r.sleep()
        if -0.2<e_drone.Error[0]<0.2 and -0.2<e_drone.Error[1]<0.2 and -0.2<e_drone.Error[2]<0.2 :
            break
        else:
            while not rospy.is_shutdown():
                e_drone.pid()
                r.sleep()
        
        # e_drone.pid()
        # if(e_drone.drone_position[0] < points_array[i][0]+0.2 and e_drone.drone_position[0] > points_array[i][0]-0.2
        #        and e_drone.drone_position[1] < points_array[i][1]+0.2 and e_drone.drone_position[1] > points_array[i][1]-0.2 and
        #        e_drone.drone_position[2] < points_array[i][2]+0.2 and e_drone.drone_position[2] > points_array[i][2] - 0.2):
        #         #print(e_drone.drone_position)
        #         # receive_image()
        #         break
        time.sleep()
    go_to_goal()
        # break