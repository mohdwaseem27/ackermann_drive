#!/usr/bin/env python
import rospy
import roslib
from math import sin, cos, tan, pi
from time import time

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#############################################################################
class AckDrive:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("ackermann_drive")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)


        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform


        # internal data
        self.wheel_circumference = pi * 0.075  # 0.075 is wheel diameter
        self.wheels_x_distance = 0.235
        self.MAX_STEERING_ANGLE = 0.415
        self.max_rpm = 330
        self.steering_angle = 0
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0


        # subscriptions
        self.lmotor_pub = rospy.Publisher("lmotor_pwm", Int32, queue_size=10)
        self.rmotor_pub = rospy.Publisher("rmotor_pwm", Int32, queue_size=10)
        self.steering_pub = rospy.Publisher("steering_angle", Int32, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmdvelCallback)

    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.getrpm()
            r.sleep()

    #############################################################################
    def getrpm(self):
        #convert m/s to m/min
        linear_vel_x_mins = self.linear_x * 60;
        linear_vel_y_mins = self.linear_y * 60;

        #convert rad/s to rad/min
        angular_vel_z_mins = self.angular_z * 60;

        wheels_x_distance = 0.235
        tangential_vel = angular_vel_z_mins * ((wheels_x_distance / 2));

        wheel_circumference = pi * 0.075
        x_rpm = linear_vel_x_mins / wheel_circumference;
        y_rpm = linear_vel_y_mins / wheel_circumference;
        tan_rpm = tangential_vel / wheel_circumference;


        #calculate for the target motor RPM and direction
        #left motor
        self.left_motor = x_rpm - y_rpm - tan_rpm;
        self.left_motor = self.constrain(self.left_motor, -self.max_rpm, self.max_rpm);

        #right motor
        self.right_motor = x_rpm + y_rpm + tan_rpm;
        self.right_motor = self.constrain(self.right_motor, -self.max_rpm, self.max_rpm);

        self.steering_angle = self.constrain(self.angular_z, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        servo_steering_angle = self.map(self.steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE, pi, 0) * (180/pi)

        self.lmotor_pub.publish(self.left_motor)
        self.rmotor_pub.publish(self.right_motor)
        self.steering_pub.publish(servo_steering_angle)



    def cmdvelCallback(self, cmd_msg):
        self.linear_x = cmd_msg.linear.x
        self.angular_z = cmd_msg.angular.z;

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def map(self, x , in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



    #############################################################################

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        ackdrive = AckDrive()
        ackdrive.spin()
    except rospy.ROSInterruptException:
        pass
