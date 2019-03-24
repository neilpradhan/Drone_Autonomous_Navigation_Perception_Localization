#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position

# Current goal (global state)
goal = None

def goal_callback(msg):
    global goal

    # RViz's "2D Nav Goal" publishes z=0, so add some altitude if needed.
    if msg.pose.position.z == 0.0:
        msg.pose.position.z = 0.4

    rospy.loginfo('New goal set:\n%s', msg)
    goal = msg

def publish_cmd(goal):
    # Need to tell TF that the goal was just generated
    goal.header.stamp = rospy.Time.now()

    if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', goal.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = goal_odom.header.frame_id

    cmd.x = goal_odom.pose.position.x
    cmd.y = goal_odom.pose.position.y
    cmd.z = goal_odom.pose.position.z

    roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))

    cmd.yaw = math.degrees(yaw)

    pub_cmd.publish(cmd)


rospy.init_node('navgoal3')
sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main():
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        if goal:
            publish_cmd(goal)
        rate.sleep()

if __name__ == '__main__':
    main()
