#!/usr/bin/env python

import rospy, math, time, os
from ar_track_alvar_msgs.msg import AlvarMarkers

arData = {"ID":[], "DZ":[], "DX":[]}

def callback(msg):
    global arData
    arData["ID"] = []
    arData["DZ"] = []
    arData["DX"] = []

    for i in msg.markers:
        arData["ID"].append(i.id)
        arData["DZ"].append(i.pose.pose.position.z)
        arData["DX"].append(i.pose.pose.position.x)

rospy.init_node('ar_detect')
rospy.Subscriber('ar_pose_marker', AlavarMarkers, callback)

rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
print("===========Start AR-Tag Detecting...")
cam_exposure(100)
index = 0

while not rospy.is_shutdown():

    for i in range(len(arData["ID"])):
        ar_id = arData["ID"][i]
        z_pose = arData["DZ"][i]
        x_pose = arData["DX"][i]

        print("%d - ID : %d Distance_Z,X: %.2f, %.2f" % (index, ar_id, z_pose, x_pose))
        print("=============================")
        index = index + 1

    time.sleep(0.5)



