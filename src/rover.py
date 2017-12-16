#!/usr/bin/python2
import math
import rospy
import message_filters

#ros imports
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#custom imports
from polyfill import roverOperation

#globals
obstableAvoidanceThreshold = 2.0
roverState = roverOperation.intermediate

def initTwist():
    command = Twist()

    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    return command

def shutdownRoutine():
    print("deploying solar panels...")
    #activate solar servos
    rospy.sleep(2)
    print("finished deploying solar panels...")

def basicAvoidance(scan):
    # find laser scanner properties (min scan angle, max scan angle, scan angle increment)
    #http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
    sensorMaxAngle = scan.angle_max
    sensorMinAngle = scan.angle_min
    angleIncrement = scan.angle_increment

    defualtVelocity = 5.0
    newBearing = 0.0


    #only read scans left to right in the givin angle increments
    minimumScannedDistance = scan.range_max
    scanTheta = sensorMinAngle
    for currentScan in scan.ranges:
        if scanTheta > (-math.pi)/2 and scanTheta < 0 and currentScan < obstableAvoidanceThreshold:
            newBearing = 1.0 #turn left
        elif scanTheta < (math.pi)/2 and scanTheta >= 0 and currentScan < obstableAvoidanceThreshold:
            newBearing = -1.0 #turn right
        
        if currentScan < minimumScannedDistance:
            minimumScannedDistance = currentScan

        scanTheta += angleIncrement

    newMovementCommand = initTwist()
    newMovementCommand.linear.x = defualtVelocity * min(1.0, minimumScannedDistance/obstableAvoidanceThreshold)
    newMovementCommand.angular.z = newBearing

    return newMovementCommand

def callback(scan,odom):
    if roverState == roverOperation.alignment:
        print("alignment alg")
    elif roverState == roverOperation.intermediate:
        newMovementCommand = basicAvoidance(scan)
    elif roverState == roverOperation.advance:
        print("advanced alg")
    else:
        print("basic alg")

    roverPublish.publish(newMovementCommand)

if __name__ == "__main__":
    rospy.init_node('rover', disable_signals=False, log_level=rospy.DEBUG)

    # subscribe to laser scan message
    subScanner = message_filters.Subscriber('base_scan', LaserScan)

    # subscribe to odometry message    
    subOdom = message_filters.Subscriber('odom', Odometry)

    # synchronize laser scan and odometry data
    ts = message_filters.TimeSynchronizer([subScanner, subOdom], 10)
    ts.registerCallback(callback)

    roverPublish = rospy.Publisher('cmd_vel',Twist)

    rospy.on_shutdown(shutdownRoutine)

    rospy.spin()