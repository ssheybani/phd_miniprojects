#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String, Float32MultiArray
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# from trajectory_msgs.msg import JointTrajectoryPoint as TrajType
from egg_sticks_game.msg import TrajType
from egg_sticks_game.msg import ForceType
from geometry_msgs.msg import Point
import math, random

import helper_funcs, trajectory_tools
from trajectory_tools import Trajectory, rms
from helper_funcs import roots2ipd, ipd2roots, norm_sampler, rk4_step, trajectory_generator, traj_max_acc, ushaped_f, slide_f, sampler, bound

# from helper_funcs import ushaped_f, rk4_step
import numpy as np

# Global variables are read from the config file
from config import *

from environment import egg_floating


# Reference Trajectory
traj_max_amp = traj_max_amp
rel_amps = [0.3, 0.6, 0.6, 0.4, 0.4]; amp_sum = np.sum(rel_amps)
rel_amps = [item/amp_sum for item in rel_amps]
n_refs = 3 # How many different trajectories do we want to include in training

traj_max_f = .5; w_max = 2*np.pi*traj_max_f

traj_obj = Trajectory(tstep=tstep, duration=duration)
traj_spec1 = traj_obj.generate_traj_spec(rel_amps, traj_max_amp, traj_max_f)
fc_req_max = traj_obj.get_max_force(traj_spec1, egg_mass, egg_fric)
egg_bnd = [fc_req_max/10, egg_ub] #[0.01, 0.2] #Note that ushaped_f() should be modified if the egg_bnd changes.
egg1 = egg_floating(egg_mass, egg_fric, egg_bnd, egg_single_step_penalty)
_f_l, _f_f = traj_obj.ideal_force_profile(traj_spec1, egg1, egg_bnd)
_ideal_rms = rms(_f_l, _f_f)
_ideal_fl_rms = rms(_f_l)
time1, traj = traj_obj.generate(traj_spec1)


T_SETTLE = 2. #The preparation time after which the trajectory reaches its normal amplitudes
# n_workers = 5 # Number of processors used for computation


time1, trajs, traj_specs = traj_obj.generate_random(3, rel_amps, traj_max_amp, traj_max_f, egg1, egg_bnd, n_deriv=n_rsamples-1, ret_specs=True)

# Second iteration for setting the dependent variables: egg_bnd and trajs
fc_req_max = traj_obj.get_max_force(traj_specs[0], egg_mass, egg_fric)
egg_bnd = [egg_lb, egg_ub] #[0.01, 0.2] 
egg1 = egg_floating(egg_mass, egg_fric, egg_bnd, egg_single_step_penalty)
_f_l, _f_f = traj_obj.ideal_force_profile(traj_specs[0], egg1, egg_bnd)
_ideal_rms = rms(_f_l, _f_f)
_ideal_fl_rms = rms(_f_l)

time1, trajs, traj_specs = traj_obj.generate_random(3, rel_amps, traj_max_amp, traj_max_f, egg1, egg_bnd, n_deriv=n_rsamples-1, ret_specs=True)
traj_mask = slide_f(time1, T_SETTLE)
for i, traj in enumerate(trajs):
    traj = traj*traj_mask # Prepare the first second of the trajectory
    trajs[i] = traj
    
time2, test_traj, test_tr_specs = traj_obj.generate_random(1, rel_amps, traj_max_amp, traj_max_f, egg1, egg_bnd, n_deriv=n_rsamples-1, ret_specs=True)
test_traj = test_traj[0]
test_traj = test_traj*traj_mask  

# time2 = np.asarray(time2, dtype=float)
time2 = time2.tolist()
test_traj = test_traj.tolist()
# test_traj = np.asarray(test_traj, dtype=float)


# Create subscribers and publishers


# mrkr_pub = rospy.Publisher('visualization_marker_array', MarkerArray,  queue_size=10)

ref_pub = rospy.Publisher('ref', TrajType, queue_size=1)
refslc0_pub = rospy.Publisher('ref_slc0', Float32MultiArray, queue_size=10)
refslc1_pub = rospy.Publisher('ref_slc1', Float32MultiArray, queue_size=10)

# markerArray = MarkerArray()
ref_msg = TrajType()
refslc0_msg = Float32MultiArray(); refslc1_msg = Float32MultiArray()
force_msg = ForceType()


if __name__ == '__main__':


    rospy.init_node('organizer_node', anonymous=False)
    fps = 1./tstep
    rate = rospy.Rate(fps) #ros spins 10 frames per second

    

    step=0;
    while not rospy.is_shutdown() and step<len(time2):
        t = time2[step]
        ref_msg.pos = test_traj[0][step]
        ref_msg.vel = test_traj[1][step]
        ref_msg.acc = test_traj[2][step]
        ref_msg.time = t 
        ref_pub.publish(ref_msg)

        ref_i0 = step-ref_slice0; ref_i1 = step+ref_slice1
        if ref_i0 <0:
            ref_i0 = 0
        if ref_i1 >= len(time2):
            ref_i1 = len(time2)
        refslc0_msg.data = test_traj[0][ref_i0:step]
        refslc1_msg.data = test_traj[0][step:ref_i1]
        refslc0_pub.publish(refslc0_msg); refslc1_pub.publish(refslc1_msg)

        step+=1
        rate.sleep()