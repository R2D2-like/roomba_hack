#!/usr/bin/env python3
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from project.srv import WavingLeftRight
from project.srv import GetGoalPoint#GetCoordinate

import tf2_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion


class SimpleController:
    def __init__(self):
        #rospy.init_node('simple_controller', anonymous=True)
        
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



class ActionGoal:
    def __init__(self):
        #self.ps_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()  # action serverの準備ができるまで待つ

    def set_goal(self, x, y, yaw):
        self.goal = MoveBaseGoal()  # goalのメッセージの定義
        self.goal.target_pose.header.frame_id = 'map'  # マップ座標系でのゴールとして設定
        self.goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
        
        # ゴールの姿勢を指定
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)  # 回転はquartanionで記述するので変換
        self.goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
 
    # def send_topic(self):
    #     #self.ps_pub.publish(self.goal)
    def send_action(self, duration=30.0):
        self.action_client.send_goal(self.goal)  # ゴールを命令
        result = self.action_client.wait_for_result(rospy.Duration(duration))
        return result


if __name__=='__main__':
    rospy.init_node('task1_manager', anonymous=True)


    #go to wave detection point (step1)
    simple_controller = SimpleController()
    



    #send goal (step5)
    rate = rospy.Rate(10)
    ac = ActionGoal()
        
    rospy.sleep(1)

    
    ac.set_goal(1.0, 1.5, 0.0)
    res = ac.send_action()
    simple_controller.stop()

    ac.set_goal(2, 1.5, 0.0)
    res = ac.send_action()
    simple_controller.stop()

    ac.set_goal(3, 1.5, 0.0)
    res = ac.send_action()
    simple_controller.stop()
    
    ac.set_goal(3.5, 1.5, 0.0)
    res = ac.send_action()
    simple_controller.stop()

    ac.set_goal(3.5, 3, 0.0)
    res = ac.send_action()
    simple_controller.stop()

    ac.set_goal(3.5, 4, 0.0)
    res = ac.send_action()
    simple_controller.stop()

    rate.sleep()



    

     


