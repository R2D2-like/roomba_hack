#!/usr/bin/env python3

import rospy
import tf2_ros
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class ActionGoal:
    def __init__(self):
        #self.ps_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()  # action serverの準備ができるまで待つ

        #rospy.sleep(1.0)
 
    # def set_goal(self, x, y, yaw):
    #     self.goal = PoseStamped() # goalのメッセージの定義
    #     self.goal.header.stamp = rospy.Time.now() # 現在時刻
    #     self.goal.header.frame_id = 'map' # マップ座標系でのゴールとして設定
    

        # # ゴールの姿勢を指定
        # self.goal.pose.position.x = x
        # self.goal.pose.position.y = y
        # print(x)
        # q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)  # 回転はquartanionで記述するので変換
        # self.goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        # print(self.goal)

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

def Main():
    rospy.init_node('target_goal', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            t = tfBuffer.lookup_transform('map','object_frame',rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue

        print('{0:.2f}, {1:.2f}, {2:.2f}'.format(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ))
        print('{0:.2f}, {1:.2f}, {2:.2f}, {3:.2f}'.format(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ))
        ac = ActionGoal()
        ac.set_goal(t.transform.translation.x-0.3, t.transform.translation.y-0.3, 0.0)
        res = ac.send_action()
        print(res)
      

        rate.sleep()

if __name__ == '__main__':
    Main()