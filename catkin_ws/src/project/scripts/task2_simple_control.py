#!/usr/bin/env python3
import numpy as np
from std_msgs.msg import String
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=10)

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

    def turn_left(self, yaw, yawrate=0.3):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def tf_spin(self):
        vel = Twist()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        flag = True
        while flag:
            try:
                t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                if abs(t.transform.rotation.w)>0.04:
                    vel.linear.x = 0.0
                    vel.angular.z = 0.2
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


    def stop(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

    def get_yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]

if __name__=='__main__':
    pub = rospy.Publisher('/task2/manager/trigger', String, queue_size=10)
    rospy.sleep(7)


    simple_controller = SimpleController()
    try:
        simple_controller.go_straight(1.4)
        rospy.sleep(1.0)
        simple_controller.tf_spin()
        simple_controller.stop()
        rospy.sleep(1.0)
    except rospy.ROSInitException:
        pass

    pub.data = 'task2'
    pub.publish('task2/manager/trigger')