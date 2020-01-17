#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
from egg_sticks_game.msg import TrajType
from egg_sticks_game.msg import ForceType
from geometry_msgs.msg import Point
import math, random

from config import *

from helper_funcs import ushaped_f, rk4_step
import numpy as np

class egg_floating():
    
    # No Global variables used
    # Imports:
    # ushaped_f
    
    def __init__(self, mass, fric, bnd, ss_penalty):
#         bnd[0] # Minimum force to keep the egg from falling
#         bnd[1] # Maximum force that will cause the egg to break

        self.mass = mass 
        self.fric = fric
        self.ss_penalty = ss_penalty #Single-step penalty: positive number

        self.normf_eval = ushaped_f(bnd[0], bnd[1], ss_penalty).apply
        
        self.x = 0.
        self.v = 0.
        self.net_f = 0.
        self.penalty = 0. # The values go from 0 to inf
        

    def reset(self):
        self.x = 0.
        self.v = 0.
        self.net_f = 0.
        self.penalty = 0. # The values go from 0 to -inf
        
        
    def eggSys(self, x, u, t=0):
        return np.array([x[1],(-self.fric*x[1]+x[2])/self.mass, u])
    
    
    def update_penalty(self, fup_obj, fdown_obj):
        #         # break constraint
        if fup_obj.f <0 or fdown_obj.f <0:# The egg is about to be dropped
            self.penalty += 2*self.ss_penalty
#             self.penalty +=  
        if fup_obj.f <fdown_obj.f:
            n_force_obj = fup_obj
        else:
            n_force_obj = fdown_obj
        self.penalty += self.normf_eval(n_force_obj.f) #normf_eval returns positive numbers
        
        return n_force_obj
        

    def update_state(self, net_df, t, tstep):

        eggState = [self.x, self.v, self.net_f]
#         u = net_df
        self.x, self.v, self.net_f = rk4_step(self.eggSys, eggState, net_df, t, tstep)


# # Callback functions
def CbFup(f_up):
    global force_up
    force_up = f_up


def CbFdown(f_down):
    global force_down
    force_down = f_down




if __name__ == '__main__':


    # Create publishers and subscribers
    # mrkr_pub = rospy.Publisher('visualization_marker_array', MarkerArray,  queue_size=10)
    cursor_pub = rospy.Publisher('cursor', TrajType, queue_size=1)
    nforce_pub = rospy.Publisher('nforce', ForceType, queue_size=1)

    fup_sub = rospy.Subscriber('force_up', ForceType, CbFup)
    fdown_sub = rospy.Subscriber('force_down', ForceType, CbFdown)

    # Create the initial messages
    cursor_msg = TrajType()
    nforce_msg = ForceType()

    # markerArray = MarkerArray()

    force_up, force_down = ForceType(),ForceType()
    force_up.f = 0.; force_up.fdot = 0.;
    force_down.f = 0.; force_down.fdot = 0.;

    # Initialize the egg
    egg_bnd = [egg_lb, egg_ub] #[0.01, 0.2] 
    egg1 = egg_floating(egg_mass, egg_fric, egg_bnd, egg_single_step_penalty)


    # Setup rospy
    rospy.init_node('env_node', anonymous=False)
    fps = 1./tstep
    rate = rospy.Rate(fps) 

    # step=0;
    t = 0.
    while not rospy.is_shutdown():

        # Apply the forces on the egg
        nforce_msg = egg1.update_penalty(force_up, force_down)
        egg1.update_state(force_up.fdot-force_down.fdot, t, tstep)

        # Publish normal force
        nforce_pub.publish(nforce_msg)

        # Publish cursor data
        cursor_msg.pos = egg1.x
        cursor_msg.vel = egg1.v
        cursor_msg.acc = (egg1.net_f-egg1.fric*egg1.v)/egg1.mass 
        cursor_msg.time = 0.
        cursor_pub.publish(cursor_msg)

        # step+=1
        t +=tstep
        rate.sleep()