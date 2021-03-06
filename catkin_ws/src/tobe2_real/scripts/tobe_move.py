#!/usr/bin/python3

from threading import Thread
import rospy
import math
import numpy as np
from tobe2_real.tobe import Tobe
from tobe2_real.walker import WalkFunc
from tobe2_real.walker import Walker
from tobe2_real.stand import Stand
from geometry_msgs.msg import Vector3


if __name__ == "__main__":
    rospy.init_node("Tobe Robot")
    rospy.sleep(1)

    rospy.loginfo("Instantiating Tobe Client")
    tobe = Tobe()
    rospy.loginfo("Instantiating Standing Protocol for Tobe")
    stand = Stand(tobe)

    while not rospy.is_shutdown():
        rospy.sleep(1)
