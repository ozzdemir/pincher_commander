#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import String

waist_angle=0
rightelbow_angle=0
rightshoulder_angle=0
rightpit_angle=0
lhand_distance=0
profile_angle=0
rhx_coordinate=0
rhy_coordinate=0
rhz_coordinate=0
angles_ready2publish=[0,0,0,0,0]
def initialise():
    rospy.init_node('pinchercommander_node',anonymous=True) #node name
    run() # To prevent the thread from exiting

def waist_callback(data):
    global waist_angle
    waist_angle =float(data.data)
def rightelbow_callback(data):
    global rightelbow_angle
    rightelbow_angle =float(data.data)
def rightshoulder_callback(data):
    global rightshoulder_angle
    rightshoulder_angle =float(data.data)
def rightpit_callback(data):
    global rightpit_angle
    rightpit_angle =float(data.data)
def lhand_callback(data):
    global lhand_distance
    lhand_distance =float(data.data)
def profile_callback(data):
    global profile_angle
    profile_angle =float(data.data)

def rhx_callback(data):
    global rhx_coordinate
    rhx_coordinate =float(data.data)
def rhy_callback(data):
    global rhy_coordinate;
    rhy_coordinate =float(data.data)
def rhz_callback(data):
    global rhz_coordinate;
    rhz_coordinate =float(data.data)    

def run():
    rate=rospy.Rate(20)# 20 hz
    rospy.Subscriber('body_publisher/waist',String,waist_callback)
    rospy.Subscriber('body_publisher/rightelbow',String,rightelbow_callback)
    rospy.Subscriber('body_publisher/rightshoulder',String,rightshoulder_callback)
    rospy.Subscriber('body_publisher/rightpit',String,rightpit_callback)
    rospy.Subscriber('body_publisher/lhand_distance', String, lhand_callback)
    rospy.Subscriber('body_publisher/profile', String, profile_callback)

    lift_commander = rospy.Publisher('arm_shoulder_lift_joint/command',Float64,queue_size=10)
    pan_commander  = rospy.Publisher('arm_shoulder_pan_joint/command',Float64,queue_size=10)
    elbow_commander = rospy.Publisher('arm_elbow_flex_joint/command',Float64,queue_size=10)
    wrist_commander = rospy.Publisher('arm_wrist_flex_joint/command',Float64,queue_size=10)
    gripper_commander = rospy.Publisher('gripper_joint/command',Float64,queue_size=10)

    while not rospy.is_shutdown():
     	angles_ready2publish[0]=3.14-np.deg2rad(waist_angle)
     	angles_ready2publish[1]=-2.9+2*np.deg2rad(profile_angle)
     	angles_ready2publish[2]=3.14-np.deg2rad(rightelbow_angle)
     	angles_ready2publish[3]=2.4-2*np.deg2rad(rightpit_angle)
     	angles_ready2publish[4]=0.8-lhand_distance/500
        lift_commander.publish(angles_ready2publish[3])# -2.07/2.19
        pan_commander.publish(angles_ready2publish[1])#  -2.44/2.44
        elbow_commander.publish(angles_ready2publish[2]/2)# -2.47/2.37
        wrist_commander.publish(angles_ready2publish[2]/2)# -1.71/1.67
        gripper_commander.publish(angles_ready2publish[4])# -1.04/0.87
        rate.sleep()

if __name__ == '__main__':
    try:
        initialise()
    except rospy.ROSInterruptException:
        pass
