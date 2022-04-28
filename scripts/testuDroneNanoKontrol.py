import rospy
import numpy as np
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import Thrust
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Joy



class Attitude_test:

    def __init__(self):
        self.att = AttitudeTarget()
        self.joy_axes = np.zeros(8)
        self.joy_buttons = np.zeros(8)
        self.att_setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)
        self.thrust = Thrust()
        self.thrust_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=100)
        self.joy_sub = rospy.Subscriber("/nanokontrol/nanokontrol", Joy, self.joy_callback, queue_size=rospy.get_param("~queue_size", None))

    def joy_callback(self, data):
        self.joy_axes = data.axes
        self.joy_buttons = data.buttons 

    def send_att(self, attitude, thrust):
        self.rate = rospy.Rate(10)  # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = Quaternion(*quaternion_from_euler(attitude[0], attitude[1],
                                                                 attitude[2]))
        self.att.thrust = thrust
        self.att.type_mask = 7  # ignore body rate
        print(self.joy_axes)
        self.att.header.stamp = rospy.Time.now()
        self.att_setpoint_pub.publish(self.att)

        self.rate.sleep()
#        print("sending attitude setpoint")

    def send_thrust(self, thrust):
        self.thrust.header.stamp = rospy.Time.now()
        self.thrust.thrust = thrust
        self.thrust_pub.publish(self.thrust)
        self.rate.sleep()
 #       print("sending thrust")

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def set_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print('service set_mode call failed: %s. Offboard Mode could not be set.' % e)

    def setDisarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def setAutoLandMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print('service set_mode call failed: %s. Autoland Mode could not be set.' % e)

    def run(self):
        while not rospy.is_shutdown():

            thrust = self.joy_axes[3]

            attitude = (self.joy_axes[0], self.joy_axes[1], self.joy_axes[2])
            self.send_att(attitude, thrust)



rospy.init_node('test_node', anonymous=True)
attitude_test = Attitude_test()
attitude_test.run()
