#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import utm

x0 = None
y0 = None


def callback(data):
    global x0
    global y0
    x, y, h, t = utm.from_latlon(data.latitude, data.longitude)

    if x0 is None:
        x0 = x
        y0 = y

    dist = math.sqrt(math.pow(x-x0,2) + math.pow(y-y0,2))
    print(dist, x0, y0, x, y)

    # rospy.loginfo(rospy.get_caller_id() + "I heard %f %f", x,y)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/rover/ublox/fix", NavSatFix, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
