#!/usr/bin/env python

#subscribing to aruco markers topic wh has msg type as marker array that is why we are converitng it to posetamped and then we put transform between camera link and map frame  and then publish or brodcast the transform to view it in Rviz

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
from tf.transformations import quaternion_from_euler

# Current goal (global state)
goal = None

rad2deg = math.pi/180.0

def goal_callback(msg):
    global goal

    #print("Aruco detected:", msg)
    goal = msg

def transform_aruco(goal):
    #goal.header.stamp = rospy.Time.now()
    pose = PoseStamped()#as the marker array is differnet from pose stamped
    pose.pose.position.x = goal.markers[0].pose.pose.position.x
    pose.pose.position.y = goal.markers[0].pose.pose.position.y
    pose.pose.position.z = goal.markers[0].pose.pose.position.z
    pose.pose.orientation.x = goal.markers[0].pose.pose.orientation.x
    pose.pose.orientation.y = goal.markers[0].pose.pose.orientation.y
    pose.pose.orientation.z = goal.markers[0].pose.pose.orientation.z
    pose.pose.orientation.w = goal.markers[0].pose.pose.orientation.w
    pose.header.frame_id = goal.markers[0].header.frame_id
    pose.header.stamp = goal.markers[0].header.stamp
    marker_id = goal.markers[0].id

    if not tf_buf.can_transform('cf1/base_link', pose.header.frame_id, pose.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/base_link' % pose.header.frame_id)
        return
        
    # check if transform is available    

    #if not tf_buf.can_transform('cf1/base_link', pose.header.frame_id, pose.header.stamp):
        #rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/base_link' % pose.header.frame_id)
        #return

    pose_base_link = tf_buf.transform(pose,'cf1/base_link')

    pose_base_link.header.stamp = pose.header.stamp
    print("Base Link frame data:", pose_base_link)

    if not tf_buf.can_transform('cf1/odom', pose_base_link.header.frame_id, pose_base_link.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % pose_base_link.header.frame_id)
        return

    pose_odom = tf_buf.transform(pose_base_link,'cf1/odom')

    print("Odom frame data:", pose_odom)
    pose_odom.header.stamp = rospy.Time.now()

    if not tf_buf.can_transform('map', pose.header.frame_id, pose.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % pose.header.frame_id)
        return

    pose_world = tf_buf.transform(pose,'map')
    print("Map frame data:", pose_world)

    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'aruco/detected' + str(marker_id)
    t.transform.translation.x = pose_world.pose.position.x
    t.transform.translation.y = pose_world.pose.position.y
    t.transform.translation.z = pose_world.pose.position.z
    t.transform.rotation.x = pose_world.pose.orientation.x
    t.transform.rotation.y = pose_world.pose.orientation.y
    t.transform.rotation.z = pose_world.pose.orientation.z
    t.transform.rotation.w = pose_world.pose.orientation.w
    broadcaster.sendTransform(t) 






    

    

rospy.init_node('final1')
sub_goal = rospy.Subscriber('/aruco/markers', MarkerArray, goal_callback)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.StaticTransformBroadcaster()
#t = TransformStamped()
#t.header.frame_id = 'cf1/base_link'
#t.child_frame_id = 'camera_link'
#t.transform.translation.x = 0.01
#t.transform.translation.y = 0.0
#t.transform.translation.z = 0.02
#roll, pitch, yaw = 0, 0, 90
#(t.transform.rotation.x,
# t.transform.rotation.y,
# t.transform.rotation.z,
# t.transform.rotation.w) = quaternion_from_euler(roll * rad2deg,
#                                                 pitch * rad2deg,
#                                                 yaw * rad2deg)
#broadcaster = tf2_ros.StaticTransformBroadcaster()
#broadcaster.sendTransform(t) 

def main():
 

    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
       
        if goal:
            transform_aruco(goal)
        rate.sleep()

if __name__ == '__main__':
    main()
