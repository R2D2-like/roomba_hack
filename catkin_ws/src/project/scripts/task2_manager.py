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
from PIL import Image

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

    def go_straight(self, dis, velocity=0.2):
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

    def turn_left(self, yaw, yawrate=0.8):
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
    def send_action(self, duration=50.0):
        self.action_client.send_goal(self.goal)  # ゴールを命令
        result = self.action_client.wait_for_result(rospy.Duration(duration))
        return result

    def cancel(self):
        self.action_client.cancel_all_goals()


if __name__=='__main__':
    rospy.init_node('task2_manager', anonymous=True)


    rospy.sleep(5)

    #go to wave detection point (step1)


    simple_controller = SimpleController()
    try:
         simple_controller.go_straight(1.3)
         simple_controller.turn_left(90)
    except rospy.ROSInitException:
         pass
    '''
    ac.set_goal(3.5, 5, 0.0)
    res = ac.send_action()
    simple_controller.stop()
    ac.cancel()
    ac.set_goal(0, 0, 90.0)
    res = ac.send_action()
    simple_controller.stop()
    ac.cancel()
    '''
    mask_ankle_trigger_pub = rospy.Publisher('/mask/ankle/trigger', String, queue_size=10)
    ac = ActionGoal()
    #reqest waving person result(step2)
    waving_person = rospy.ServiceProxy('/wave_detection', WavingLeftRight)
    res  = waving_person()
    rospy.sleep(0.5)
    #induce masking ankle(step3)
    mask_ankle_trigger_pub = rospy.Publisher('/mask/ankle/trigger', String, queue_size=10)
    #while not rospy.is_shutdown():

    print(res.left_or_right)
    left_or_right = res.left_or_right
    waving_person_str = String()
    waving_person_str.data = res.left_or_right
    mask_ankle_trigger_pub.publish(waving_person_str)
    rospy.sleep(0.05)


    #reqest goal coordinate(step4)
    get_coordinate = rospy.ServiceProxy('/get_coordinate', GetGoalPoint)
    res = get_coordinate()

    #send goal (step5)
    rate = rospy.Rate(10)
    # ac = ActionGoal()
    '''
    estimete_x_min = 0.8
    estimete_x_max = 1.7
    estimete_x = 1.5

    if (res.x<estimete_x_min) or (estimete_x_max<res.x):
            x = estimete_x
            y = res.y
            print("goal(" + str(x) + "," + str(y) + ")")
            ac.set_goal(x, y, 0.0)
            res = ac.send_action()
    else:
            x = res.x
            y = res.y
            print("goal(" + str(x+0.5) + "," + str(y) + ")")
            ac.set_goal(x+0.5, y, 0.0)
            res = ac.send_action()
    '''

    estimete_x_min = 0.3
    estimete_x_max = 1.0
    estimete_x = 1.2
    print(res.x)

    if (res.x<estimete_x_min) or (estimete_x_max<res.x):
            x = estimete_x
            y = res.y
            if left_or_right == 'left':
                print("goal(" + str(x) + "," + str(y-0.2) + ")")
                ac.set_goal(x, y-0.2, 0.0)#if left y-0.2, if right y+0.2
            else:
                print("goal(" + str(x) + "," + str(y+0.2) + ")")
                ac.set_goal(x, y, 0.0)#if left y-0.2, if right y+0.2
            res = ac.send_action()
            simple_controller.stop()
    else:
            x = res.x
            y = res.y
            if left_or_right == 'left':
                print("goal(" + str(x+0.35) + "," + str(y-0.2) + ")")
                ac.set_goal(x+0.35, y-0.2, 0.0)#if left y-0.2, if right y+0.2
            else:
                print("goal(" + str(x+0.35) + "," + str(y+0.2) + ")")
                ac.set_goal(x+0.35, y, 0.0)#if left y-0.2, if right y+0.2
            # print("goal(" + str(x+0.4) + "," + str(y) + ")")
            # ac.set_goal(x+0.35, y-0.2, 0.0)
            res = ac.send_action()
            simple_controller.stop()


    # print("goal(" + str(x+0.3) + "," + str(y) + ")")
    # ac.set_goal(x-0.3, y, 0.0)
    # res = ac.send_action()
    print(res)
    rate.sleep()








