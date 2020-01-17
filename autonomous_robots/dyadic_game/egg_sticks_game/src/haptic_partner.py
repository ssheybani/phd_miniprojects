#!/usr/bin/env python

# Agent 1: pushes towards the positive direction

from __future__ import print_function
import rospy
from std_msgs.msg import String
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# from trajectory_msgs.msg import JointTrajectoryPoint as TrajType
from egg_sticks_game.msg import TrajType
from egg_sticks_game.msg import ForceType
from geometry_msgs.msg import Point
import numpy as np
from numpy import array as narr

from config import *
from controllers_config import *

import helper_funcs
from helper_funcs import bound


class agent_dual_mode():
    
    # Global variables used:
    # n_w, traj_max_amp, tstep
    # import np, narr
    
    @staticmethod
    def map_params(paramset):
        # returns three lists: controller weights, two switching policies (from S to T and vice versa).
        
        sw0 = paramset[:n_w]#np.zeros(n_w) #
        sw1 = paramset[n_w: 2*n_w]#np.zeros(n_w) #
        
        w = [stblrlist[int(paramset[n_w*2])], ctrlist[int(paramset[n_w*2+1])]]#[ctrlist[0],stblrlist[0]] #
    
    
        return w, sw0, sw1
    
    
    def __init__(self, paramset):
     # w_max is the maximum angular frequency that the ref will have.
    
        self.w, self.sw1, self.sw0, = agent_dual_mode.map_params(paramset)
        
#         self.w_max = w_max
        
        self.role = 0; # initial role: stabilize
        self.evidence = -1; # evidence (as in diffusion decision model)
        self.ref_period = agent_decision_delay #refractory period in terms of time steps
        self.counter = 0; 
        self.feature_vec = []
        self.current_f = 0.
        
#         self.scaling_vec = [1., tstep, tstep**2, 1., tstep, tstep**2, traj_max_amp, traj_max_amp*tstep, traj_max_amp]
#         self.scaling_vec = [1., 1., 1., 1., 1., 1., 1, 1., 1]
    
    
    def reset(self):
        pass
#         self.cumError = 0
    
    @staticmethod
    def sensor2feature(sensor_data):
	ref = narr([sensor_data['ref'].pos, sensor_data['ref'].vel, sensor_data['ref'].acc])
	pos = np.array([sensor_data['cursor'].pos, sensor_data['cursor'].vel, sensor_data['cursor'].acc])
	err = ref -pos
	nforce = np.array([sensor_data['nforce'].f, sensor_data['nforce'].fdot])
        
    	# Make sure all features are in the scale of traj_max_amp. Special      
	features = np.concatenate( ( ref, err, traj_max_amp*nforce, [traj_max_amp] ))

	return features


    def _update_role(self): # Complete this method
        # state is a vector containing the current state of the task
        # It looks like np.array(ftr_r, ftr_p, traj_max_amp* ftr_normf, [traj_max_amp])
        
        # Toggle the role of agent if necessary
        if self.counter <=0: # If not in refractory period:...
            if self.role == 1:
                self.evidence = np.dot(self.sw0, self.feature_vec)
                if self.evidence >0:
                    self.role = 0
                    self.counter = self.ref_period

            elif self.role == 0:
                self.evidence = np.dot(self.sw1, self.feature_vec)
                if self.evidence >0:
                    self.role = 1
                    self.counter = self.ref_period

        elif self.counter >0:
            self.counter -=1

        
    def generate_force(self, sensor_data, verbose=False):
        
        # Run the decision unit for one step and read the role from its output
        # Apply the appropriate controller given the role
    
        
        self.feature_vec = self.sensor2feature(sensor_data)
        self._update_role()
        
        fdot = np.dot(self.w[self.role], self.feature_vec)
        
        # bound fdot
        fdot = bound(fdot, fdot_bound)

        # bound force
        force = bound(self.current_f +fdot*tstep, [0., f_bound], symm=False)
        # Recalculate fdot
        fdot = (force-self.current_f)/tstep;
        
        self.current_f = force

        if verbose:
            return (force, fdot, self.feature_vec)
        return (force, fdot)


def Cb_ref(ref_data):
	global sensor_data
	sensor_data['ref'] = ref_data
	

def Cb_cursor(cursor_data):
	global sensor_data
	sensor_data['cursor'] = cursor_data
	

def Cb_nforce(nforce_data):
	global sensor_data
	sensor_data['nforce'] = nforce_data
	



if __name__ == '__main__':

    # The agent parameters, i.e. switching policies and controllers
    switchCondpos = [0,0,0, 0,1,0, 0, 0, -0.1]
    switchCondneg = [0,0,0, 0,-1,0, 0, 0, -0.1]

    test_paramset = switchCondpos+switchCondneg+[1, 1]


    # Create publishers and subscribers
    # mrkr_pub = rospy.Publisher('visualization_marker_array', MarkerArray,  queue_size=10)
    force_pub = rospy.Publisher('force_up', ForceType, queue_size=1)

    ref_sub = rospy.Subscriber('ref', TrajType, Cb_ref)
    cursor_sub = rospy.Subscriber('cursor', TrajType, Cb_cursor)
    nforce_sub = rospy.Subscriber('nforce', ForceType, Cb_nforce)

    # Create the initial messages
    # markerArray = MarkerArray()
    force_msg = ForceType()

    sensor_data = {'ref': [], 'cursor':[], 'nforce':[]}


    # Setup rospy
    rospy.init_node('agent1_node', anonymous=False)
    fps = 1./tstep
    rate = rospy.Rate(fps) 

    agent = agent_dual_mode(test_paramset)

    # step=0;
    while not rospy.is_shutdown():
    	# Form feature vector, using sensor_data
    	if not sensor_data['ref'] or not sensor_data['cursor'] or not sensor_data['nforce']:
    		continue

        # The virtual agent's force
        out_force, out_df = agent.generate_force(sensor_data)
        # The force resulted by the dynamics of the cursor, i.e. f = bv +ma (v,a can be extracted from sensor_data)

        force_msg.f = out_force; force_msg.fdot = out_df
        force_pub.publish(force_msg)

        # step+=1
        rate.sleep()
