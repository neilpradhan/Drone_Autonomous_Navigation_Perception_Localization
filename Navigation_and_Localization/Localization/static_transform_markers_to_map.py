#!/usr/bin/env python

import sys
import math
import json
from aruco_msgs.msg import MarkerArray
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3
import tf
import tf2_geometry_msgs

with open('/home/song/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/awesome.world.json') as f:
    world = json.load(f)
def markers_map(m):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'aruco/marker' + str(m['id'])
    t.transform.translation = Vector3(*m['pose']['position'])
    roll,pitch,yaw= m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))

    return t

if __name__ == "__main__":
    rospy.init_node('markersatmap')
    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    transforms = [markers_map(m) for m in world['markers']]
    broadcaster2.sendTransform(transforms)
rospy.spin()
