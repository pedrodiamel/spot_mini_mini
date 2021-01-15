#!/usr/bin/env python

#---- The line above is crucial since
#---- this is a Python script !!!


#------------------------------------#
#---- Python - SpotMiniMini walk ----#
#------------------------------------#

"""
DESCRIPTION: Creates node that sets the walk parameters and makes the SpotMiniMini
             simulation in Gazebo walk based on a 12 point Bezier Curve.

SUBSCRIBERS: /command - receives a character to indicate direction of movement
             /mini_cmd - store high level information of movement 

PUBLISHERS: spot/front_left_hip_position_controller/command
            spot/front_left_leg_position_controller/command
            spot/front_left_foot_position_controller/command
            spot/front_right_hip_position_controller/command
            spot/front_right_leg_position_controller/command
            spot/front_right_foot_position_controller/command
            spot/back_left_hip_position_controller/command
            spot/back_left_leg_position_controller/command
            spot/back_left_foot_position_controller/command
            spot/back_right_hip_position_controller/command
            spot/back_right_leg_position_controller/command
            spot/back_right_foot_position_controller/command
"""


#---- Importing Libraries and Packages ----#

# from __future__ import division
import os
import rospy
import numpy as np
from mini_ros.msg import MiniCmd
from std_msgs.msg import String, Float64
import copy
import time

import sys

import rospkg 
rospack = rospkg.RosPack()


sys.path.append(rospack.get_path('mini_ros') + '/../')

sys.path.append('../../')

from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait


#---- Program Parameters ----#

# Controller Params
STEPLENGTH_SCALE = 0.06
Z_SCALE_CTRL = 0.12
RPY_SCALE = 0.6
SV_SCALE = 0.1
CHPD_SCALE = 0.0005
YAW_SCALE = 1.5

# AGENT PARAMS
CD_SCALE = 0.05
SLV_SCALE = 0.05
RESIDUALS_SCALE = 0.03
Z_SCALE = 0.05
# Filter actions
alpha = 0.7
# Added this to avoid filtering residuals
# -1 for all


#---- Class to Command SpotMiniMini Gazebo Model ----#

class SpotCommander():
    """
    This class stablishes all publishers and subscribers needed to
    move the SpotMiniMini robot inside Gazebo. It also sets the walking
    parameters received via the /command topic to make the robot walk
    in the desired direction. The class publishes the messages to move
    the robot in simulation.
    """

    def __init__(self):
        """
        Creates the node that makes the SpotMiniMini in Gazebo walk. Also
        creates a message to store high level data of the robot movement.
        Initializes the SpotMiniMini model features, that will define the
        robot gait and also other simulation parameters. Sets all robot 
        parameters for movement.
        """

        # Initializing walk node:
        rospy.init_node('Walk', anonymous=True)
        
        # Creating message and setting parameters:
        self.mini_cmd = MiniCmd()

        self.mini_cmd.x_velocity = 0.0
        self.mini_cmd.y_velocity = 0.0
        self.mini_cmd.rate = 0.0
        self.mini_cmd.roll = 0.0
        self.mini_cmd.pitch = 0.0
        self.mini_cmd.yaw = 0.0
        self.mini_cmd.z = 0.0
        self.mini_cmd.motion = "Stop"
        self.mini_cmd.movement = "Stepping"
        # Fixed parameters:
        self.BaseStepVelocity = 0.1
        self.StepVelocity = self.BaseStepVelocity

        self.BaseSwingPeriod = 0.2
        self.SwingPeriod = self.BaseSwingPeriod

        self.BaseClearanceHeight = 0.04
        self.BasePenetrationDepth = 0.005
        self.ClearanceHeight = self.BaseClearanceHeight
        self.PenetrationDepth = self.BasePenetrationDepth

        # Parameters for movement
        self.StepLength = 0.045
        self.LateralFraction = 0.0
        self.YawRate = 0.0

        # Load the SpotMiniMini features model:
        self.LoadSpot()

        # Define Publishers:
        self.SetPublishers()

        # Define Subscribers:
        self.SetSubscribers()
        
        # Get curret time as simulation start time:
        self.time = rospy.get_time()
        # Print on command line to notify:
        print("READY TO GO!")

        # Debug Variables
        self.command_letter = "S"


    def LoadSpot(self):
        """
        Creates a SpotModel and a BezierGait object to enable
        the use of the key features for gait generation for the
        SpotMiniMini model simulation. Also sets some simulation
        parameters.
        """

        # Set seed:
        seed = 0
        np.random.seed(seed)

        # Load Spot Model:
        self.spot = SpotModel()

        # Set simulation parameters:
        self.rate_value = 100
        self.rate = rospy.Rate(self.rate_value) # Simulation Frequency in Hz
        self.dt = 1/self.rate_value

        # Get gait generation parameters:
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait(dt=self.dt)

    

    #---- Define Motion Publishers ----#

    def SetPublishers(self):
        """
        Define all publishers needed for a movement node.
        """

        # Information srtorage message publisher:
        self.pub_MiniCmd = rospy.Publisher('mini_cmd', MiniCmd, queue_size=1)

        # Defining publishers to move the motors position:
        # Front Left Leg:
        pub_FLH = rospy.Publisher("spot/front_left_hip_position_controller/command", Float64, queue_size=10)
        pub_FLL = rospy.Publisher("spot/front_left_leg_position_controller/command", Float64, queue_size=10)
        pub_FLF = rospy.Publisher("spot/front_left_foot_position_controller/command", Float64, queue_size=10)
        # Front Right Leg:
        pub_FRH = rospy.Publisher("spot/front_right_hip_position_controller/command", Float64, queue_size=10)
        pub_FRL = rospy.Publisher("spot/front_right_leg_position_controller/command", Float64, queue_size=10)
        pub_FRF = rospy.Publisher("spot/front_right_foot_position_controller/command", Float64, queue_size=10)
        # Back or Rear Left Leg:
        pub_BLH = rospy.Publisher("spot/back_left_hip_position_controller/command", Float64, queue_size=10)
        pub_BLL = rospy.Publisher("spot/back_left_leg_position_controller/command", Float64, queue_size=10)
        pub_BLF = rospy.Publisher("spot/back_left_foot_position_controller/command", Float64, queue_size=10)
        # Back or Rear Right Leg:
        pub_BRH = rospy.Publisher("spot/back_right_hip_position_controller/command", Float64, queue_size=10)
        pub_BRL = rospy.Publisher("spot/back_right_leg_position_controller/command", Float64, queue_size=10)
        pub_BRF = rospy.Publisher("spot/back_right_foot_position_controller/command", Float64, queue_size=10)

        # Store all publishers in the class attribute:
        self.move_publishers = [pub_FLH, pub_FLL, pub_FLF,
                                pub_FRH, pub_FRL, pub_FRF,
                                pub_BLH, pub_BLL, pub_BLF,
                                pub_BRH, pub_BRL, pub_BRF]

    
    #---- Define Motion Subscribers ----#

    def SetSubscribers(self):
        """
        Define all subscribers for any movement node.
        """

        self.sub_cmd = rospy.Subscriber('mini_cmd', MiniCmd, self.mini_cmd_cb)
        self.sub_guide = rospy.Subscriber("/command", String, self.guide_cb)


    #---- Subscribers Callback functions ----#

    def mini_cmd_cb(self, mini_cmd): # We are not actually using this callback yet...
        """ Reads the desired Minitaur command and passes it for execution

            Args: mini_cmd
        """
        try:
            # Update mini_cmd
            self.mini_cmd = mini_cmd
            # log input data as debug-level message
            rospy.logdebug(mini_cmd)
        except rospy.ROSInterruptException:
            pass
    

    def guide_cb(self, msg):
        """ 
        Receives a command message to guide the robot walking action by 
        setting the walk parameters.
        Following convention is used:
            F : move forward
            L : turn left
            R : turn right
            S : stop
            
        Input: 
            String msg: command message
        Output:
            None
        """
        try:            
            # Set parameters based on message content
            if (msg.data == "F"):
                self.SwingPeriod = 0.2
                self.StepVelocity = 0.50
                self.StepLength = 0.045
                self.LateralFraction = 0.0
                self.YawRate = 0.0
                self.ClearanceHeight = 0.045
                self.PenetrationDepth = 0.003	  
            elif (msg.data == "L"):
                self.SwingPeriod = 0.2
                self.StepVelocity = 0.001
                self.StepLength = 0.011
                self.LateralFraction = 0.0
                self.YawRate = 2.0
                self.ClearanceHeight = 0.045
                self.PenetrationDepth = 0.003	
            elif (msg.data == "R"):
                self.SwingPeriod = 0.2
                self.StepVelocity = 0.001
                self.StepLength = 0.011
                self.LateralFraction = 0.0
                self.YawRate = -2.0
                self.ClearanceHeight = 0.045
                self.PenetrationDepth = 0.003
            elif (msg.data == "S"):
                self.SwingPeriod = 0.2
                self.StepVelocity = 0.001
                self.StepLength = 0.0
                self.LateralFraction = 0.0
                self.YawRate = 0.0
                self.ClearanceHeight = 0.045
                self.PenetrationDepth = 0.003		

            # Move MiniMini Model
            # self.Move(StepLength, LateralFraction,YawRate)

            # Store command letter
            self.command_letter = msg.data
		
        except rospy.ROSInterruptException:
            pass


    def Move(self):
        """
        Creates the robot gait based on a 12 point Bezier curve
        and the inverse kinematics of the robot.

        Input:
            float StepLength: the robots step length in meters
            float LateralFraction: the robots lateral fraction of movement
            float YawRate: the robots yaw rate for the movement
        
        Output:
            None
        """

        # Setting foot contact:
        # We set it as all 0 since we are not monitoring this variable.
        # 0 means no contact and 1 means contact.
        contacts = [0,0,0,0]

        # Elapsed time:
        dt = rospy.get_time() - self.time
        # Current time:
        self.time = rospy.get_time()

        # Fixed variables for gait:
        # We considered all zero since we are not monitoring these variables.
        pos = np.array([0.0, 0.0, 0.0])
        orn = np.array([0.0, 0.0, 0.0])
        
        # Store swing parameter:
        self.bzg.Tswing = self.SwingPeriod
        
        # Generate Gait:
        self.T_bf = self.bzg.GenerateTrajectory(self.StepLength, self.LateralFraction,
                                                self.YawRate, self.StepVelocity,
                                                self.T_bf0, self.T_bf,
                                                self.ClearanceHeight,
                                                self.PenetrationDepth,
                                                contacts, dt)

        joint_angles = self.spot.IK(orn, pos, self.T_bf)

        # Publish motor angles configuration for robot:
        self.PublishAngles(joint_angles.reshape(-1))


    def PublishAngles(self, target):
        """
        Publishes the motor messages to move all motors.
        """
        # Loop to make all publishers publish:
        for i in range(12):
            # Log the message so we can debug:
            #rospy.loginfo(self.lowCmd_msg.motorCmd[i].position)
            # Publish the message:
            self.move_publishers[i].publish(target[i])




def main():
    """ The main() function. """
    mini_commander = SpotCommander()
    rate = rospy.Rate(600.0) # Frequency of the cycle to make reading uniform (Hz)
     
    while not rospy.is_shutdown():
        # This is called continuously. Has timeout functionality too
        #mini_commander.Move()
        # rate.sleep() # Wait to maintain the frequency constant
        # Move MiniMini Model
        mini_commander.Move()
        rate.sleep()
        # rospy.spin()


if __name__ == '__main__':
    try:
        time.sleep(15.) # Wait for robot to fall down and stabilize (in seconds)
        main()
    except rospy.ROSInterruptException:
        pass
