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

import tf2_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


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
                    #print('a')

                else:
                    self.stop()
                    rospy.sleep(1)
                    flag = False
                    print('bb')
                    break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rospy.sleep(0.1)
                continue


    def tf_go(self):
        vel = Twist()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        flag = True
        while flag:
            try:
                t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                if t.transform.translation.x > 1.2 :
                    vel.linear.x = 0.3
                    vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(vel)
                    #rospy.sleep(0.1)
                    #print('a')
                
                else:
                    self.stop()
                    rospy.sleep(1)
                    flag = False
                    print('bb')
                    break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rospy.sleep(0.1)
                continue



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

def callback(data):
     #go to wave detection point (step1)


    simple_controller = SimpleController()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    action_client.wait_for_server()  # action serverの準備ができるまで待つ

    mask_ankle_trigger_pub = rospy.Publisher('/mask/ankle/trigger', String, queue_size=10)
    ac = ActionGoal()
    action_client.cancel_all_goals()
    #reqest waving person result(step2)
    rospy.wait_for_service('/wave_detection')
    waving_person = rospy.ServiceProxy('/wave_detection', WavingLeftRight)
    res  = waving_person()
    print(res)
    rospy.sleep(0.5)
    #induce masking ankle(step3)
    #mask_ankle_trigger_pub = rospy.Publisher('/mask/ankle/trigger', String, queue_size=10)
    #while not rospy.is_shutdown():

    print(res.left_or_right)
    left_or_right = res.left_or_right
    waving_person_str = String()
    waving_person_str.data = res.left_or_right
    print(waving_person_str.data)
    mask_ankle_trigger_pub.publish(waving_person_str)
    rospy.sleep(0.05)


    #reqest goal coordinate(step4)
    rospy.wait_for_service('/get_coordinate')
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
    estimete_x_max = 1.1
    estimete_x = 1.0
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
            print(res)
            simple_controller.stop()
            #simple_controller.tf_go()
            #simple_controller.stop()
            simple_controller.go_straight(0.1)
            simple_controller.stop()
    else:
            x = res.x
            y = res.y
            if left_or_right == 'left':
                print("goal(" + str(x+0.35) + "," + str(y-0.2) + ")")
                if y < 4.3:
                    y = 4.3
                ac.set_goal(1.1, y-0.2, 0.0)#if left y-0.2, if right y+0.2
                
                #simple_controller.go_straight(1.1-(x+0.25)+0.35)
            else:
                print(y)
                if 5.6 < y:
                    y = 5.6
                if 5.2> y:
                    y = 5.0
                print("goal(" + str(x+0.35) + "," + str(y+0.2) + ")")
                ac.set_goal(1.1, y+0.2, 0.0)#if left y-0.2, if right y+0.2
                #simple_controller.go_straight(1.1-(x+0.25)+0.35)
            # print("goal(" + str(x+0.4) + "," + str(y) + ")")
            # ac.set_goal(x+0.35, y-0.2, 0.0)
            res = ac.send_action()
            #simple_controller.tf_go()
            simple_controller.stop()
            simple_controller.go_straight(0.1)
            simple_controller.stop()
            cnt = 0
            flag = True
            simple_conyroll_flag = False
            # while flag:
            #     tmp_time = rospy.Time.now()
            #     while (rospy.Time.now().secs - tmp_time.secs) < 5.0:
            #         try:
            #             t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
            #             if t.transform.translation.x > 1.5 :
            #                 if cnt <= 3:
            #                     print(t.transform.translation.x)
            #                     simple_controller.go_straight(0.05,-0.2)
            #                     rospy.sleep(0.5)
            #                     res2 = ac.send_action()
            #                     simple_controller.stop()
            #                     print('res2 : ' + str(res2))
            #                     action_client.cancel_all_goals()
            #                 else:
            #                     flag = False
            #                 cnt += 1
            #             else:
            #                 flag = False
            #                 simple_conyroll_flag = True
            #                 break
            #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #             print(e)
            #             rospy.sleep(0.1)
            #             continue

            # if simple_conyroll_flag:
            #     if left_or_right == 'left':
            #         simple_controller.tf_spin()
            #         simple_controller.turn_left(30)
            #         simple_controller.tf_go()
            #         simple_controller.stop()
            #         simple_controller.go_straight(0.5)
            #     else:
            #         simple_controller.tf_spin()
            #         simple_controller.turn_right(30)
            #         simple_controller.tf_go()
            #         simple_controller.stop()
            #         simple_controller.go_straight(0.5)

                    




    # print("goal(" + str(x+0.3) + "," + str(y) + ")")
    # ac.set_goal(x-0.3, y, 0.0)
    # res = ac.send_action()
    print(res)
    rate.sleep()


if __name__=='__main__':
    rospy.init_node('task2_manager', anonymous=True)
    sub = rospy.Subscriber('/task2/manager/trigger', String, callback)

    rospy.spin()
    #rospy.sleep(5)










