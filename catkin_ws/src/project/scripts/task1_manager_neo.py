#!/usr/bin/env python3
#from tracemalloc import reset_peak
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import tf2_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import time
from std_srvs.srv import Empty
#from project.srv import DetectionTrigger2
from std_msgs.msg import Float64MultiArray, MultiArrayLayout



class SimpleController:
    def __init__(self):
        #rospy.init_node('simple_controller', anonymous=True)

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
                if abs(t.transform.rotation.w)<0.99:
                    vel.linear.x = 0.0
                    vel.angular.z = -0.2
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
        rospy.sleep(0.5)

    def tf_spin_backward(self):
        vel = Twist()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        flag = True
        while flag:
            try:
                t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                if abs(t.transform.rotation.w)>0.02:
                    vel.linear.x = 0.0
                    vel.angular.z = -0.2
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
        rospy.sleep(0.5)




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
def clear_map():
    rospy.wait_for_service('/move_base/clear_costmaps')
    clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    clear_res = clear_srv()
    print('claer map')
    rospy.sleep(2)

def add_map():
    rospy.wait_for_service('/add_map')
    add_srv = rospy.ServiceProxy('/add_map', Empty)
    add_res = add_srv()

if __name__=='__main__':
    rospy.init_node('task1_manager', anonymous=True)
    action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    action_client.wait_for_server()  # action serverの準備ができるまで待つ


    #go to wave detection point (step1)
    simple_controller = SimpleController()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)



    #send goal (step5)
    rate = rospy.Rate(10)
    ac = ActionGoal()
    mode = 0

    rospy.sleep(1)
    clear_map()

    print("start task1")
    simple_controller.turn_left(30)


    #detection = rospy.ServiceProxy('/clip/detection_trigger', DetectionTrigger2)
    detection = rospy.ServiceProxy('/clip/detection_trigger', Empty)
    rospy.wait_for_service('/clip/detection_trigger')
    #roslist = Float64MultiArray()
    #roslist.data = [0,1,0,0,2,0,4,0]
    #roslist.layout = MultiArrayLayout()
    #req = DetectionTrigger2()
    #req.BeforeCounter = roslist
    #detection.BeforeCounter = roslist


    # print(list(detection.BeforeCounter.data))
    det_res = detection()
    #rospy.sleep(3)
    simple_controller.turn_right(30)
    #detection.BeforeCounter.data = det_res.AfterCounter.data
    det_res = detection()
    #rospy.sleep(3)
    #clear
    clear_map()
    rospy.wait_for_service('/move_base/clear_costmaps')
    clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    clear_res = clear_srv()
    print('clear')
    #add
    add_map()

    ac.set_goal(1.5, 1.5, 0.0)
    #send_time = rospy.Time.now()
    res1 = ac.send_action()
    simple_controller.stop()
    print('res1 : ' + str(res1))
    action_client.cancel_all_goals()
    flag = res1
    roop_flag = True
    tmp_time = rospy.Time.now()
    while (rospy.Time.now().secs - tmp_time.secs) < 5.0:
        try:
            t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
            if t.transform.translation.x < 1.0 :
                print(t.transform.translation.x)
                simple_controller.tf_spin_forward()
                simple_controller.turn_left(20)
                # clear_map()
                # add_map()
                ac.set_goal(1.5, 2.6, 0.0)
                res1 = ac.send_action()
                simple_controller.stop()
                print('res2-2 : ' + str(res1))
                action_client.cancel_all_goals()
                flag = res1

                tmp_time2 = rospy.Time.now()
                while (rospy.Time.now().secs - tmp_time2.secs) < 5.0:
                    try:
                        if roop_flag:
                            t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                            if t.transform.translation.x < 1.0 :
                                print(t.transform.translation.x)
                                simple_controller.tf_spin_forward()
                                simple_controller.turn_right(20)
                                ac.set_goal(1.5, 0.4, 0.0)
                                res1 = ac.send_action()
                                simple_controller.stop()
                                print('res2-3 : ' + str(res1))
                                action_client.cancel_all_goals()
                                flag = res1
                                tmp_time3 = rospy.Time.now()
                                while (rospy.Time.now().secs - tmp_time3.secs) < 5.0:
                                    try:
                                        t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                                        if t.transform.translation.x < 1.0 :
                                            print(t.transform.translation.x)
                                            flag = False
                                            roop_flag = False
                                            break
                                        else:
                                            mode = 2
                                            roop_flag = False
                                            break
                                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                                        print(e)
                                        rate.sleep()
                                        continue

                            else:
                                mode = 1
                                roop_flag = False
                                break

                        else:
                            break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        print(e)
                        rate.sleep()
                        continue
            else:
                break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue
    if not flag:
        #clear
        simple_controller.tf_spin_forward()
        clear_map()
        rospy.wait_for_service('/move_base/clear_costmaps')
        clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_res = clear_srv()
        cnt = 0
        for i in range(2):
            res1_2 = ac.send_action()
            simple_controller.stop()
            print('res1_2 : ' + str(res1_2))
            action_client.cancel_all_goals()
            if res1_2:
                break
            cnt += 1
    if mode ==  0:
        simple_controller.tf_spin_forward()
        simple_controller.turn_left(30)
        det_res = detection()
        #rospy.sleep(3)
        simple_controller.turn_left(30)
        det_res = detection()
        #rospy.sleep(3)
        simple_controller.tf_spin_forward()
        simple_controller.turn_right(30)
        det_res = detection()
        #rospy.sleep(3)
        simple_controller.turn_left(30)
        det_res = detection()
        #rospy.sleep(3)
    elif mode == 1:
        simple_controller.tf_spin_forward()
        simple_controller.turn_right(30)
        det_res = detection()
        #rospy.sleep(3)
        simple_controller.turn_right(30)
        det_res = detection()
        #rospy.sleep(3)
        simple_controller.turn_left(60)
        det_res = detection()
    else:
        simple_controller.tf_spin_forward()
        simple_controller.turn_left(30)
        det_res = detection()
        #rospy.sleep(3)
        simple_controller.turn_left(30)
        det_res = detection()
        #rospy.sleep(3)
        simple_controller.turn_right(60)
        det_res = detection()



    #clear
    clear_map()
    rospy.wait_for_service('/move_base/clear_costmaps')
    clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    clear_res = clear_srv()
    print('clear')
    #add
    add_map()

    ac.set_goal(3.5, 2.2, 0.0)
    #send_time = rospy.Time.now()
    res2 = ac.send_action()
    simple_controller.stop()
    print('res2 : ' + str(res2))
    action_client.cancel_all_goals()
    flag = res2
    roop_flag = True
    tmp_time = rospy.Time.now()
    while (rospy.Time.now().secs - tmp_time.secs) < 5.0:
        try:
            t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
            if t.transform.translation.x < 1.0 :
                print(t.transform.translation.x)
                simple_controller.tf_spin_forward()
                simple_controller.go_straight(0.15,-0.3)
                simple_controller.turn_left(20)
                ac.set_goal(3.5, 3.1, 0.0)
                print('3')
                res1 = ac.send_action()
                simple_controller.stop()
                print('res2-2 : ' + str(res1))
                action_client.cancel_all_goals()
                flag = res1

                tmp_time2 = rospy.Time.now()
                while (rospy.Time.now().secs - tmp_time2.secs) < 5.0:
                    try:
                        if roop_flag:
                            t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                            if t.transform.translation.x < 1.0 :
                                print(t.transform.translation.x)
                                simple_controller.tf_spin_forward()
                                simple_controller.go_straight(0.15,-0.3)
                                simple_controller.turn_right(20)
                                ac.set_goal(3.5, 1.2, 0.0)
                                res1 = ac.send_action()
                                simple_controller.stop()
                                print('res2-3 : ' + str(res1))
                                action_client.cancel_all_goals()
                                flag = res1
                                tmp_time3 = rospy.Time.now()
                                while (rospy.Time.now().secs - tmp_time3.secs) < 5.0:
                                    try:
                                        t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
                                        if t.transform.translation.x < 1.0 :
                                            print(t.transform.translation.x)
                                            flag = False
                                            roop_flag = False
                                            break
                                        else:
                                            mode = 2
                                            roop_flag = False
                                            break
                                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                                        print(e)
                                        rate.sleep()
                                        continue

                            else:
                                mode = 1
                                roop_flag = False
                                break
                        else:
                            break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        print(e)
                        rate.sleep()
                        continue
            else:
                break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue
    if not flag:
        #clear
        clear_map()
        simple_controller.tf_spin_forward()
        simple_controller.go_straight(0.15,-0.3)
        rospy.wait_for_service('/move_base/clear_costmaps')
        clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_res = clear_srv()
        cnt = 0
        simple_controller.tf_spin_forward()
        for i in range(4):
            ac.set_goal(3.5, 2.2, 0.0)
            res2_2 = ac.send_action()
            simple_controller.stop()
            print('res2_4 : ' + str(res2_2))
            action_client.cancel_all_goals()
            if res2_2:
                break
            cnt += 1


    simple_controller.tf_spin_backward()

    #detection.BeforeCounter = det_res.AfterCounter
    det_res = detection()
    #rospy.sleep(3)
    simple_controller.turn_right(90)

    #clear
    clear_map()
    rospy.wait_for_service('/move_base/clear_costmaps')
    clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    clear_res = clear_srv()
    ac.set_goal(3.5, 5, 0.0)
    cnt = 0
    for i in range(5):
        if cnt >= 2:
            simple_controller.tf_spin_forward()
            simple_controller.turn_left(90)
            simple_controller.go_straight(4.0)
            break
        res3 = ac.send_action(60)
        simple_controller.stop()
        print('res3 : ' + str(res3))
        action_client.cancel_all_goals()

        if res3:
            break
        cnt += 1
        simple_controller.tf_spin_forward()
        simple_controller.turn_left(90)











    '''
    simple_controller.stop()




    ac.set_goal(1.5, 1.5, 0.0)
    res1 = ac.send_action()
    simple_controller.stop()
    print(res1)
    action_client.cancel_all_goals()

    if not res1:
        simple_controller.go_straight(0.3,-0.2)
        rospy.sleep(1)
        ac.set_goal(1.5, 2, 0.0)
        res1_2 = ac.send_action()
        simple_controller.stop()
        print(res1_2)
        action_client.cancel_all_goals()


    # ac.set_goal(2, 1.5, 0.0)
    # res = ac.send_action()
    # simple_controller.stop()

    # ac.set_goal(3, 1.5, 0.0)
    # res = ac.send_action()
    # simple_controller.stop()

    rospy.sleep(1)
    ac.set_goal(3.5, 2.5, 0.0)
    res2 = ac.send_action(45)
    simple_controller.stop()
    print(res2)
    action_client.cancel_all_goals()


    if not res2:
        simple_controller.go_straight(0.3,-0.2)
        rospy.sleep(1)
        ac.set_goal(3.5, 3, 0.0)
        res2_2 = ac.send_action()
        simple_controller.stop()
        print(res2_2)
        action_client.cancel_all_goals()

    # ac.set_goal(3.5, 3, 0.0)
    # res = ac.send_action()
    # simple_controller.stop()
    rospy.sleep(1)
    ac.set_goal(3.5, 4, 0.0)
    res3 = ac.send_action()
    simple_controller.stop()
    print(res3)

    if not res3:
        simple_controller.go_straight(0.3,-0.2)
        rospy.sleep(1)
        ac.set_goal(3.5, 4, 0.0)
        res3_2 = ac.send_action()
        simple_controller.stop()
        print(res3_2)
        action_client.cancel_all_goals()

    rate.sleep()

    '''








