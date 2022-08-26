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


class TopicGoal:
    def __init__(self):
        self.ps_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.sleep(1.0)
 
    def set_goal(self, x, y, yaw):
        self.goal = PoseStamped() # goalのメッセージの定義
        self.goal.header.stamp = rospy.Time.now() # 現在時刻
        self.goal.header.frame_id = 'map' # マップ座標系でのゴールとして設定

        # ゴールの姿勢を指定
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        print(x)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)  # 回転はquartanionで記述するので変換
        self.goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        print(self.goal)
 
    def send_topic(self):
        self.ps_pub.publish(self.goal)

def Main():
    rospy.init_node('object_goal', anonymous=True)
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
        tg = TopicGoal()
        tg.set_goal(t.transform.translation.x-0.1, t.transform.translation.y-0.1, 0.0)
        tg.send_topic()


        rate.sleep()

if __name__ == '__main__':
    Main()