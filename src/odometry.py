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
class ACKTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("odometry")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)


        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform

        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame



        # internal data
        self.wheel_circumference = pi * 0.075  # 0.075 is wheel diameter
        self.wheels_x_distance = 0.235
        self.angular_vel_z = 0
        self.MAX_STEERING_ANGLE = 0.415
        self.steering_angle = 0
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        self.heading = 0
        self.x_pos = 0
        self.y_pos = 0
        self.rpm1 = 0
        self.rpm2 = 0
        self.then = rospy.Time.now()

        # subscriptions
        rospy.Subscriber("lwheel", Int32, self.lwheelCallback)  #rpm of left motor
        rospy.Subscriber("rwheel", Int32, self.rwheelCallback)  #rpm of right motor
        rospy.Subscriber("/cmd_vel", Twist, self.cmdvelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            self.getvel()
            r.sleep()


    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        vel_dt = (now - self.then).to_sec()
        self.then = now
        delta_heading = self.steering_angle * vel_dt
        delta_x = (self.linear_x * cos(self.heading) - self.linear_y * sin(self.heading) * vel_dt)
        delta_y = (self.linear_x * sin(self.heading) - self.linear_y * cos(self.heading) * vel_dt)

        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += delta_heading

        quaternion = quaternion_from_euler(0, 0,self.heading)

        # publish the odom information
        self.odomBroadcaster.sendTransform(
            (self.x_pos, self.y_pos, 0),
            (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
            )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.angular_z
        self.odomPub.publish(odom)




    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        self.rpm1 = msg.data

    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        self.rpm2 = msg.data


    #############################################################################
    def getvel(self):
        avg_rps_x = (float(self.rpm1 + self.rpm2) / 2) / 60
        rospy.loginfo(avg_rps_x)
        self.linear_x = avg_rps_x * self.wheel_circumference
        self.angular_z = (self.linear_x * tan(self.steering_angle)) / self.wheels_x_distance


    def cmdvelCallback(self, cmd_msg):
        self.angular_vel_z = cmd_msg.angular.z;
        self.steering_angle = self.constrain(self.angular_vel_z, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE);

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))





    #############################################################################

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        ackTf = ACKTf()
        ackTf.spin()
    except rospy.ROSInterruptException:
        pass
