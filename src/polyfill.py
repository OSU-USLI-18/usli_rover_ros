import enum
from nav_msgs.msg import Odometry
import tf 

class roverOperation(enum.Enum):
    alignment = 0
    basic = 1
    intermediate = 2
    advance = 3

def getEuler():
    odom = Odometry()
    # find current orientation of robot based on odometry (quaternion coordinates)
    xOr = odom.pose.pose.orientation.x
    yOr = odom.pose.pose.orientation.y
    zOr = odom.pose.pose.orientation.z
    wOr = odom.pose.pose.orientation.w

    # find orientation of robot (Euler coordinates)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([xOr, yOr, zOr, wOr])
    return (roll, pitch, yaw)

def getCurrentCoor():
    odom = Odometry()

    currX = odom.pose.pose.position.x
    currY = odom.pose.pose.position.y

    return (currX,currY)