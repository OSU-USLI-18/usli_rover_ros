#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Range

rospy.init_node('sonar_scanner')
sonarScanPub = rospy.Publisher('scan',Range,queue_size=50)

count = 0
r = rospy.Rate(10.0)#adjust refresh rate curr: 10hz

while not rospy.is_shutdown():
    scan = Range() #init msg

    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = "range_frame"
    scan.radiation_type = 0 # set to UltraSound
    #scan.field_of_view = 0.0 # i dont know what to set this to right now

    scan.min_range = 0.6096 #2 feet
    scan.max_range = 1.524  #5 feet

    scan.range = 1.0 #change to recieved rover sonar data

    sonarScanPub.publish(scan)
    r.sleep()
