#!/usr/bin/env python

import rospy, math, time, os
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0

def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)


def callback(msg):
    global arData
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

rospy.init_node('ar_info_print')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
print("=== Start AR-Tag Detecting...")

cam_exposure(100)

while not rospy.is_shutdown():
    (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    print(" roll : "+ str(roll))
    print("pitch : "+ str(pitch))
    print(" yaw : "+ str(yaw))

    print(" x : " + str(round(arData["DX"],3)))
    print(" y : " + str(round(arData["DY"],3)))
    print(" z : " + str(round(arData["DZ"],3)))

    time.sleep(0.5)
