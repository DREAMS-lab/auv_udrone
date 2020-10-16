"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import RCIn, ActuatorControl
import numpy


class RC2ActuatorControl:
    mixer = numpy.array([[3, 1],
                         [0, 1],
                         [1, 1],
                         [2, 1]], dtype=float)

    rc_scaling = numpy.array([[1000, 1996], [1002, 1998], [1003, 2001], [1000, 1911]], dtype=float)

    out = numpy.array([0, 0, 0, 0], dtype=float)
    rc_in = numpy.array([0, 0, 0, 0], dtype=float)

    def conversion(self, x, i):
        x1 = self.rc_scaling[i, 0]
        x2 = self.rc_scaling[i, 1]
        val = (x - x1) / (x2 - x1)
        return 2*val-1

    def __init__(self):
        rospy.init_node('offboard_rc_actuator_control_test', anonymous=True)
        actuator_control_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
        rospy.Subscriber('/mavros/rc/in', RCIn, callback=self.rc_in_callback)

        actuator_control = ActuatorControl()
        actuator_control.group_mix = 0
        rate = rospy.Rate(20)  # Hz
        rate.sleep()

        while not rospy.is_shutdown():
            for i in range(0, 4):
                actuator_control.controls[int(self.mixer[i, 0])] = self.rc_in[i] * self.mixer[i, 1]
            actuator_control_pub.publish(actuator_control)
            rate.sleep()


    def rc_in_callback(self, msg):
        for i in range(0, 4):
            self.rc_in[i] = self.conversion(msg.channels[i], i)


if __name__ == "__main__":
    RC2ActuatorControl()
