#!/usr/bin/env python
"""
DESCRIPTION:

SUBSCRIBERS:
"""

from __future__ import division
import os
import rospy
import numpy as np
from mini_ros.msg import MiniCmd
from std_msgs.msg import String
import copy

import sys

import rospkg 
rospack = rospkg.RosPack()


sys.path.append(rospack.get_path('mini_ros') + '/../')

sys.path.append('../../')

from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv
from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait

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


class SpotCommander():
    def __init__(self):

        rospy.init_node('Walk', anonymous=True)
        self.agents = {}
        # self.movetypes = [
        #     "Forward", "Backward", "Left", "Right", "CW", "CCW", "Stop"
        # ]
        self.movetypes = ["Stop"]
        self.mini_cmd = MiniCmd()
        #self.jb = JoyButtons()
        self.mini_cmd.x_velocity = 0.0
        self.mini_cmd.y_velocity = 0.0
        self.mini_cmd.rate = 0.0
        self.mini_cmd.roll = 0.0
        self.mini_cmd.pitch = 0.0
        self.mini_cmd.yaw = 0.0
        self.mini_cmd.z = 0.0
        self.mini_cmd.motion = "Stop"
        self.mini_cmd.movement = "Stepping"
        # FIXED
        self.BaseStepVelocity = 0.1
        self.StepVelocity = self.BaseStepVelocity
        # Stock, use Bumpers to change
        self.BaseSwingPeriod = 0.2
        self.SwingPeriod = self.BaseSwingPeriod
        # Stock, use arrow pads to change
        self.BaseClearanceHeight = 0.04
        self.BasePenetrationDepth = 0.005
        self.ClearanceHeight = self.BaseClearanceHeight
        self.PenetrationDepth = self.BasePenetrationDepth

        self.load_spot()
        # mini_cmd_cb from mini_cmd topic
        self.sub_cmd = rospy.Subscriber('mini_cmd', MiniCmd, self.mini_cmd_cb)
        self.sub_guide = rospy.Subscriber("/command", String, self.guide_cb)
        self.time = rospy.get_time()
	
	# Publishers
	self.pub = rospy.Publisher('mini_cmd', MiniCmd, queue_size=1)
        print("PRONTO")

    def load_spot(self):

        self.env = spotBezierEnv(render=True,
                                 on_rack=False,
                                 height_field=False,
                                 draw_foot_path=False)

        self.env.reset()

        seed = 0
        # Set seeds
        self.env.seed(seed)
        np.random.seed(seed)

        state_dim = self.env.observation_space.shape[0]
        print("STATE DIM: {}".format(state_dim))
        action_dim = self.env.action_space.shape[0]
        print("ACTION DIM: {}".format(action_dim))
        max_action = float(self.env.action_space.high[0])
        print("RECORDED MAX ACTION: {}".format(max_action))

        self.state = self.env.reset()

        # Load Spot Model
        self.spot = SpotModel()

        self.dt = self.env._time_step

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait(dt=self.env._time_step)

    def mini_cmd_cb(self, mini_cmd):
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
	Following convention is followed:
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
        	StepLength = 0.045
        	LateralFraction = 0.0
        	YawRate = 0.0
        	self.ClearanceHeight = 0.045
        	self.PenetrationDepth = 0.003	  
	    elif (msg.data == "L"):
		self.SwingPeriod = 0.2
        	self.StepVelocity = 0.001
        	StepLength = 0.011
        	LateralFraction = 0.0
        	YawRate = 2.0
        	self.ClearanceHeight = 0.045
        	self.PenetrationDepth = 0.003	
	    elif (msg.data == "R"):
		self.SwingPeriod = 0.2
        	self.StepVelocity = 0.001
        	StepLength = 0.011
        	LateralFraction = 0.0
        	YawRate = -2.0
        	self.ClearanceHeight = 0.045
        	self.PenetrationDepth = 0.003
	    elif (msg.data == "S"):
		self.SwingPeriod = 0.2
        	self.StepVelocity = 0.001
        	StepLength = 0.0
        	LateralFraction = 0.0
        	YawRate = 0.0
        	self.ClearanceHeight = 0.045
        	self.PenetrationDepth = 0.003		

	    # Move MiniMini Model
	    self.move(StepLength, LateralFraction,YawRate)
		
        except rospy.ROSInterruptException:
            pass

    def move(self, StepLength, LateralFraction,YawRate):
        

        contacts = self.state[-4:]
        # contacts = [0,0,0,0]

        # Time
        dt = rospy.get_time() - self.time
        # print("dt: {}".format(dt))
        self.time = rospy.get_time()

        # Mexemos aqui
        pos = np.array([0.0, 0.0, 0.0])
        orn = np.array([0.0, 0.0, 0.0])
        #self.SwingPeriod = 0.2
        self.bzg.Tswing = self.SwingPeriod
        #self.StepVelocity = 0.50
        #StepLength = 0.045
        #LateralFraction = 0.0
        #YawRate = 0.0
        #self.ClearanceHeight = 0.045
        #self.PenetrationDepth = 0.003
        self.T_bf = self.bzg.GenerateTrajectory(StepLength, LateralFraction,
                                                YawRate, self.StepVelocity,
                                                self.T_bf0, self.T_bf,
                                                self.ClearanceHeight,
                                                self.PenetrationDepth,
                                                contacts, dt)

        joint_angles = self.spot.IK(orn, pos, self.T_bf)
        self.env.pass_joint_angles(joint_angles.reshape(-1))
        # Get External Observations
        # TODO
        # self.env.spot.GetExternalObservations(bzg, bz_step)
        # Step
        action = self.env.action_space.sample()
        action[:] = 0.0
        self.state, reward, done, _ = self.env.step(action)


def main():
    """ The main() function. """
    mini_commander = SpotCommander()
    rate = rospy.Rate(600.0) # Frequency of the cycle to make reading uniform (Hz)
    while not rospy.is_shutdown():
        # This is called continuously. Has timeout functionality too
        #mini_commander.move()
        rate.sleep() # Wait to maintain the frequency constant
        # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

