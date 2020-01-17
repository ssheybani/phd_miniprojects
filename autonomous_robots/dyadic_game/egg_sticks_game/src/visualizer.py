#!/usr/bin/env python

# Agent 1: pushes towards the positive direction

from __future__ import print_function
import rospy
from std_msgs.msg import String, Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# from trajectory_msgs.msg import JointTrajectoryPoint as TrajType
from egg_sticks_game.msg import TrajType
from egg_sticks_game.msg import ForceType
from geometry_msgs.msg import Point
import numpy as np
from numpy import array as narr

from config import *
from viz_config import *


mrkr_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
marker_arr = MarkerArray()


def Cb_refslc0(refslc0_msg):
    global ref_slc0
    ref_slc0 = refslc0_msg.data

def Cb_refslc1(refslc1_msg):
    global ref_slc1
    ref_slc1 = refslc1_msg.data

def Cb_ref(ref_msg):
    global ref_pos
    ref_pos = ref_msg.pos
    # ref_marker(ref_msg.pos)
	

def Cb_cursor(cursor_msg):
    global egg_pos
    egg_pos = cursor_msg.pos
    # egg_marker(cursor_msg.pos)
	

def Cb_Fup(fup_msg):
    global force_up
    force_up = fup_msg.f
    # fup_marker(fup_msg.f)


def Cb_Fdown(fdown_msg):
    global force_down
    force_down = fdown_msg.f
    # fdown_marker(fdown_msg.f)

	
def Cb_Fnormal(fnorm_msg):
    global force_normal
    force_normal = fnorm_msg.f
    # fdown_marker(fdown_msg.f)



def viz_markers():
    global marker_arr, mrkr_pub, fmeter_pt
    
    # ref slice
    refslc_mrkr = Marker()
    refslc_mrkr.type = Marker.LINE_STRIP
    refslc_mrkr.header.frame_id = "map"
    refslc_mrkr.action = Marker.ADD
    refslc_mrkr.ns = "RefSlice"

    refslc_mrkr.scale.x = viz_scale*ref_size/20
    refslc_mrkr.color.r = ref_color[0]; refslc_mrkr.color.g = ref_color[1]; refslc_mrkr.color.b = ref_color[2]
    refslc_mrkr.color.a = 1.0
    refslc_mrkr.id = 0

    slc0_len = len(ref_slc0);
    for i, item in enumerate(ref_slc0):
        pt_x = viz_scale*item; pt_y = float(-slc0_len+ i)/20;
        refslc_mrkr.points.append(Point(pt_x, pt_y, 0))
    for i, item in enumerate(ref_slc1):
        pt_x = viz_scale*item; pt_y = float(i)/20;
        refslc_mrkr.points.append(Point(pt_x, pt_y, 0))

    marker_arr.markers.append(refslc_mrkr)

    # ref
    ref_mrkr = Marker()
    ref_mrkr.type = Marker.POINTS
    ref_mrkr.header.frame_id = "map"
    ref_mrkr.action = Marker.ADD
    ref_mrkr.ns = "Ref"

    ref_mrkr.scale.x = viz_scale*ref_size
    ref_mrkr.scale.y = viz_scale*ref_size
    ref_mrkr.scale.z = viz_scale*ref_size
    ref_mrkr.color.r = 1.; #ref_mrkr.color.g = 1.; ref_mrkr.color.b = 1.
    ref_mrkr.color.a = 1.0
    ref_mrkr.id = 1

    ref_mrkr.points.append(Point(viz_scale*ref_pos, 0, 0))

    marker_arr.markers.append(ref_mrkr)
    

    #egg
    egg_mrkr = Marker()
    egg_mrkr.type = Marker.SPHERE
    egg_mrkr.header.frame_id = "map"
    egg_mrkr.action = Marker.ADD
    egg_mrkr.ns = "Egg"
    egg_mrkr.pose.position.x = viz_scale*egg_pos;
    egg_mrkr.pose.position.y = 0;
    egg_mrkr.pose.position.z = 0;

    egg_mrkr.scale.x = viz_scale*egg_dims[0]
    egg_mrkr.scale.y = viz_scale*egg_dims[1]
    egg_mrkr.scale.z = viz_scale*egg_dims[2]
    egg_mrkr.color.r = 1.; egg_mrkr.color.g = 1.; egg_mrkr.color.b = 1.
    egg_mrkr.color.a = 0.6
    egg_mrkr.id = 2

    marker_arr.markers.append(egg_mrkr)

    # force up
    fup_mrkr = Marker()
    fup_mrkr.type = Marker.ARROW
    fup_mrkr.header.frame_id = "map"
    fup_mrkr.action = Marker.ADD
    fup_mrkr.ns = "Force"


    fup_mrkr.points.append(Point(viz_scale*egg_pos-force_up, 0, 0)) #Start point
    fup_mrkr.points.append(Point(viz_scale*egg_pos, 0, 0)) #End point

    fup_mrkr.scale.x = viz_scale*0.01
    fup_mrkr.scale.y = viz_scale*0.02
    fup_mrkr.scale.z = force_up*0.2
    fup_mrkr.color.r = 0.; fup_mrkr.color.g = 1.; fup_mrkr.color.b = 0.
    fup_mrkr.color.a = 0.9
    fup_mrkr.id = 3

    marker_arr.markers.append(fup_mrkr)



    # force down
    fdown_mrkr = Marker()
    fdown_mrkr.type = Marker.ARROW
    fdown_mrkr.header.frame_id = "map"
    fdown_mrkr.action = Marker.ADD
    fdown_mrkr.ns = "Force"

    fdown_mrkr.points.append(Point(viz_scale*egg_pos+force_down, 0, 0)) #Start point
    fdown_mrkr.points.append(Point(viz_scale*egg_pos, 0, 0)) #End point

    fdown_mrkr.scale.x = viz_scale*0.01
    fdown_mrkr.scale.y = viz_scale*0.02
    fdown_mrkr.scale.z = force_down*0.2
    fdown_mrkr.color.r = 0.; fdown_mrkr.color.g = 1.; fdown_mrkr.color.b = 0.
    fdown_mrkr.color.a = 1.
    fdown_mrkr.id = 4

    marker_arr.markers.append(fdown_mrkr)


    # Normal force and its bounds

    # Background of the normal force range
    fMeterB = Marker()
    fMeterB.type = Marker.LINE_LIST
    fMeterB.header.frame_id = "map"
    fMeterB.action = Marker.ADD
    fMeterB.ns = "MeterBase"
    fMeterB.scale.x = fmeter_w
    fMeterB.color.r = fmeter_base_c[0]; fMeterB.color.g = fmeter_base_c[1]; fMeterB.color.b = fmeter_base_c[2]
    fMeterB.color.a = 1.0
    fMeterB.id = 5
    fMeterB.points.extend([Point(fmeter_pos[0], -5, 0), 
                          Point(fmeter_pos[0], 5, 0)])

    # Allowable range for normal force
    fMeter = Marker()
    fMeter.type = Marker.LINE_LIST
    fMeter.header.frame_id = "map"
    fMeter.action = Marker.ADD
    fMeter.ns = "ForceMeter"
    fMeter.scale.x = fmeter_w
    fMeter.color.r = fmeter_c[0]; fMeter.color.g = fmeter_c[1]; fMeter.color.b = fmeter_c[2]
    fMeter.color.a = 1.0
    fMeter.id = 6
    fMeter.points.extend(fmeter_pt)

    # Normal force itself
    nf_mrkr = Marker()
    nf_mrkr.type = Marker.POINTS
    nf_mrkr.header.frame_id = "map"
    nf_mrkr.action = Marker.ADD
    nf_mrkr.ns = "NForce"

    nf_mrkr.scale.x = viz_scale*nf_size
    nf_mrkr.scale.y = viz_scale*nf_size
    nf_mrkr.scale.z = viz_scale*nf_size
    nf_mrkr.color.r = nf_c[0]; nf_mrkr.color.g = nf_c[1]; nf_mrkr.color.b = nf_c[2]
    nf_mrkr.color.a = 1.0
    nf_mrkr.id = 7

    nf_mrkr.points.append(Point(fmeter_pos[0], fmeter_pos[1]+force_normal*nf_scale, 0))


    # Force meter text
    nftext_mrkr = Marker()
    nftext_mrkr.type = Marker.TEXT_VIEW_FACING
    nftext_mrkr.header.frame_id = "map"
    nftext_mrkr.action = Marker.ADD
    nftext_mrkr.ns = "NForceText"

    nftext_mrkr.scale.z = viz_scale*nftext_size
    nftext_mrkr.color.r = nf_c[0]; nftext_mrkr.color.g = nf_c[1]; nftext_mrkr.color.b = nf_c[2]
    nftext_mrkr.color.a = 1.0
    nftext_mrkr.text = 'Normal force gauge'
    nftext_mrkr.id = 8

    nftext_mrkr.pose.position.x = fmeter_pos[0]+0.3
    nftext_mrkr.pose.position.y = 0.
    nftext_mrkr.pose.position.z = 0.

    marker_arr.markers.extend([fMeterB, fMeter, nf_mrkr, nftext_mrkr])


    mrkr_pub.publish(marker_arr)



if __name__ == '__main__':


    # # Setup rospy
    rospy.init_node('viz_node', anonymous=False)
    fps = 1./tstep #0.2/tstep
    rate = rospy.Rate(fps) 

    # Subscribers for collecting game data
    egg_pos = 0. #{'ref': [], 'cursor':[], 'nforce':[]}
    force_up = 0.; force_down = 0.; force_normal = (egg_ub+egg_lb)/2
    ref_pos = 0.
    ref_slc0 = []; ref_slc1 = []

    # Normal force meter fixed variables
    f_meter0 = egg_lb*nf_scale; f_meter1 = egg_ub*nf_scale;

    fmeter_pt = [Point(fmeter_pos[0], fmeter_pos[1]+f_meter0, 0), 
                Point(fmeter_pos[0], fmeter_pos[1]+f_meter1, 0)]


    ref_sub = rospy.Subscriber('ref', TrajType, Cb_ref)
    cursor_sub = rospy.Subscriber('cursor', TrajType, Cb_cursor)
    fup_sub = rospy.Subscriber('force_up', ForceType, Cb_Fup)
    fdown_sub = rospy.Subscriber('force_down', ForceType, Cb_Fdown)
    
    refslc0_sub = rospy.Subscriber('ref_slc0', Float32MultiArray, Cb_refslc0)
    refslc1_sub = rospy.Subscriber('ref_slc1', Float32MultiArray, Cb_refslc1)

    fn_sub = rospy.Subscriber('nforce', ForceType, Cb_Fnormal)


    while not rospy.is_shutdown():
        viz_markers()
        rate.sleep()
