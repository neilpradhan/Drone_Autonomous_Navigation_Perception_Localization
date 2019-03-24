#!/usr/bin/env python
import sys
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import numpy as np


rospy.init_node('final2')
#tf_buf   = tf2_ros.Buffer()
#tf_lstn  = tf2_ros.TransformListener(tf_buf)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
#sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)
idx = 0 #current pose index
old_time=rospy.Time.now()


poses = np.array([[1.0, 0.0, 0.4,90, 0.0, 0.0 ],#1
         [3, 0.0, 0.4,90, 0, 0.0]]) #2
         
def publish_cmd():
    global idx, old_time
    #goal = PoseStamped()

    '''goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "cf1/odom"

    goal.pose.position.x = poses[idx, 0]
    goal.pose.position.y = poses[idx, 1]
    goal.pose.position.z = poses[idx, 2]
    goal.pose.orientation.z = poses[idx, 5]'''
    #pub_cmd.publish(cmd)

    #if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', goal.header.stamp):
        #rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        #return

    #goal_odom = tf_buf.transform(goal, 'cf1/odom')

    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "cf1/odom"

    cmd.x = poses[idx, 0]
    cmd.y = poses[idx, 1]
    cmd.z = poses[idx, 2]

    roll, pitch, yaw = poses[idx, 3],poses[idx, 4],poses[idx, 5]

    cmd.yaw = math.degrees(yaw)

    pub_cmd.publish(cmd)
    
    
    if(rospy.Time.now() - old_time > rospy.Duration.from_sec(5)):
      print(idx)
      if idx == 0:
          idx = 1
      elif idx==1:
          idx = 0
      old_time = rospy.Time.now()
        






def main():
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        publish_cmd()
        rate.sleep()
        
if __name__ == '__main__':
    main()
