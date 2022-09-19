#!/usr/bin/env python3
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros

class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        self.x = None
        self.y = None
        self.yaw = None
        while self.x is None:
            rospy.sleep(0.1)

    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.yaw = self.get_yaw_from_quaternion(data.pose.pose.orientation)

    def go_straight(self, dis, velocity=0.3):
        vel = Twist()
        x0 = self.x
        y0 = self.y
        while(np.sqrt((self.x-x0)**2+(self.y-y0)**2)<dis):
            vel.linear.x = velocity
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def turn_right(self, yaw, yawrate=-0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def turn_left(self, yaw, yawrate=0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def stop(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

    def get_yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]

    def tf_spin_forward(self):
        vel = Twist()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        flag = True
        while flag:
            try:
                t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                if abs(t.transform.rotation.w)<0.998:
                    vel.linear.x = 0.0
                    vel.angular.z = -0.5
                    self.cmd_vel_pub.publish(vel)
                    #rospy.sleep(0.1)
                    print('a')

                else:
                    self.stop()
                    flag = False
                    print('bb')
                    break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rospy.sleep(0.1)
                continue
    def tf_spin_backward(self):
        vel = Twist()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        flag = True
        while flag:
            try:
                t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                if abs(t.transform.rotation.w)>0.002:
                    vel.linear.x = 0.0
                    vel.angular.z = -0.5
                    self.cmd_vel_pub.publish(vel)
                    #rospy.sleep(0.1)
                    print('a')

                else:
                    self.stop()
                    flag = False
                    print('bb')
                    break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rospy.sleep(0.1)
                continue


        self.stop()

if __name__=='__main__':
    simple_controller = SimpleController()
    try:
        simple_controller.tf_spin_forward()
        # simple_controller.go_straight(1.0)
        # simple_controller.turn_left(90)
        # simple_controller.turn_right(90)
    except rospy.ROSInitException:
        pass
