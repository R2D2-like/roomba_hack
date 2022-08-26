#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def callback(pc2):
    num_of_sampling = 50
    x_sum = 0
    y_sum = 0
    z_sum = 0
    cnt = 0
    #print("test")
    for p in point_cloud2.read_points(pc2, skip_nans=True):
        if cnt >= num_of_sampling :
            break
        x_sum += p[0]
        y_sum += p[1]
        z_sum += p[2]
        cnt += 1
    x = x_sum / num_of_sampling
    y = y_sum / num_of_sampling
    z = z_sum / num_of_sampling #ave > median?

    print([x,y,z])
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera_color_optical_frame"
    t.child_frame_id = "object_frame"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    print(t)

    br.sendTransform(t)
    

rospy.init_node('pc2_xyz')  
sub = rospy.Subscriber('/camera/depth/points',PointCloud2,callback)                                                                                                                                  
rospy.spin()