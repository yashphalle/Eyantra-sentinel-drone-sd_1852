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


class Edrone():
    """docstring for Edrone"""

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        # [x_setpoint, y_setpoint, z_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        self.drone_position = [0, 0, 0]
        self.setpoint = [0, 0, 23]

        # Declaring a cmd of message type edrone_msgs and initializing values
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

        self.arm()  # ARMING THE DRONE

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
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses

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

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

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

        # ------------------------------------------------------------------------------------------------------------------------
        # Roll Error
        self.Error[0] = (self.setpoint[0]-self.drone_position[0])
        self.cmd.rcRoll = int(1500+self.Error[0]*self.Kp[0]+(
            self.Error[0]-self.PrevError[0])*self.Kd[0]+self.sumError[0]*self.Ki[0])
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
        self.cmd.rcPitch = int(1500+self.Error[1]*self.Kp[1]+(
            self.Error[1]-self.PrevError[1])*self.Kd[1]+self.sumError[1]*self.Ki[1])
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
        self.cmd.rcThrottle = int(1500+self.Error[2]*self.Kp[2]+(
            self.Error[2]-self.PrevError[2])*self.Kd[2]+self.sumError[2]*self.Ki[2])
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


'''Set points
        1)(0,0,23)
        2)(2,0,23)
        3)(2,2,23)
        4)(-2,2,23)
        5)(-2,-2,23)
        6)(2,-2,23)
        7)(2,0,23)
        8)(0,0,23)
points_array=[[0, 0, 23], [2, 0, 23], [2, 2, 23], [-2, 2, 23], [-2, -2, 23], [2, -2, 23], [2, 0, 23], [0, 0, 23]]
i = 0'''


def move():
    e_drone.pid()
    r = rospy.Rate(30)
    points_array = [[0, 0, 23], [2, 0, 23], [2, 2, 23], [-2, 2,
                                                         23], [-2, -2, 23], [2, -2, 23], [2, 0, 23], [0, 0, 23]]
    i = 0
    while i != 8:
        e_drone.setpoint = points_array[i]
        while True:
            e_drone.pid()
            if(e_drone.drone_position[0] < points_array[i][0]+0.2 and e_drone.drone_position[0] > points_array[i][0]-0.2
               and e_drone.drone_position[1] < points_array[i][1]+0.2 and e_drone.drone_position[1] > points_array[i][1]-0.2 and
               e_drone.drone_position[2] < points_array[i][2]+0.2 and e_drone.drone_position[2] > points_array[i][2] - 0.2):
                # print(e_drone.drone_position)
                break
            r.sleep()

        i += 1


if __name__ == '__main__':
    t = time.time()
    while time.time() - t < 5:
        pass
    e_drone = Edrone()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    move()
    while not rospy.is_shutdown():
        e_drone.pid()
