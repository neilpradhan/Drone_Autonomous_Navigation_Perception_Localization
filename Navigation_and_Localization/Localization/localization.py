#!/usr/bin/env python

#average the transform between map and odom when camera sees multiply markers(add previous transform)
import sys
import rospy
import json

import math
import tf
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import collections

from tf.transformations import *
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped, Transform, PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Header
from aruco_msgs.msg import MarkerArray
from crazyflie_gazebo.msg import Position

from pyquaternion import Quaternion
queue = collections.deque(maxlen = 8)

broadcaster2 = tf2_ros.StaticTransformBroadcaster()
broadcaster = tf2_ros.TransformBroadcaster()
trans_average = None

def transformation_matrix(trans):
    mat = Quaternion(
        trans.rotation.w,
        trans.rotation.x,
        trans.rotation.y,
        trans.rotation.z)
    mat = mat.transformation_matrix
    mat[0][3] = trans.translation.x
    mat[1][3] = trans.translation.y
    mat[2][3] = trans.translation.z
    return mat



def transformation(mat):
    q = Quaternion(matrix=mat)
    trans = TransformStamped()
    trans.transform.translation.x = mat[0][3]
    trans.transform.translation.y = mat[1][3]
    trans.transform.translation.z = mat[2][3]
    trans.transform.rotation.x = q[1]
    trans.transform.rotation.y = q[2]
    trans.transform.rotation.z = q[3]
    trans.transform.rotation.w = q[0]
    return trans

def multiply_transforms(trans1, trans2):
    mat1 = transformation_matrix(trans1)
    mat2 = transformation_matrix(trans2)
    return transformation(np.matmul(mat1, mat2))

def transform(msg):
    global id_list
    id_list=[]
    id = []
    for marker in msg.markers:
        tc2m = TransformStamped()
        tc2m.header.stamp = rospy.Time.now()

        if marker.id != 0:
            tc2m.header.frame_id = 'cf1/camera_link'#
            tc2m.child_frame_id = 'aruco/detected'+str(marker.id)
            tc2m.transform.translation.x = marker.pose.pose.position.x
            tc2m.transform.translation.y = marker.pose.pose.position.y
            tc2m.transform.translation.z = marker.pose.pose.position.z
            tc2m.transform.rotation.x = marker.pose.pose.orientation.x
            tc2m.transform.rotation.y = marker.pose.pose.orientation.y
            tc2m.transform.rotation.z = marker.pose.pose.orientation.z
            tc2m.transform.rotation.w = marker.pose.pose.orientation.w
            broadcaster.sendTransform(tc2m)
            id.append(marker.id)
    id_list= id
id_list =[]


def compute_avg_transform():
    print('Computing average transform')
    global trans_average
    x_avg = np.mean(np.array([transform.transform.translation.x for transform in queue]))
    y_avg = np.mean(np.array([transform.transform.translation.y for transform in queue]))
    z_avg = np.mean(np.array([transform.transform.translation.z for transform in queue]))

    quats = [ (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w) for transform in queue]
    Q = np.transpose(np.array(quats))

    M = np.dot(Q,Q.T)

    w,v = np.linalg.eig(M)
    vec = v[:,np.argmax(w)]

    mean_quat = normalize(vec)

    tr = TransformStamped()
    tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = x_avg, y_avg, 0#z_avg
    x, y, z, w = mean_quat[0], mean_quat[1], mean_quat[2], mean_quat[3]
    roll, pitch, yaw = euler_from_quaternion((x, y, z, w))
    (tr.transform.rotation.x,
    tr.transform.rotation.y,
    tr.transform.rotation.z,
    tr.transform.rotation.w) = quaternion_from_euler(0,0,yaw)

    trans_average = tr


def normalize(v):
	norm = np.linalg.norm(v)
	if norm == 0 :
		return v
	return v / norm


def main():
    global trans_average
    rospy.init_node('localization')
    rospy.Subscriber('/aruco/markers', MarkerArray, transform)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(20.0)
    found = False


    while not rospy.is_shutdown():
        try:
            # if found:
            #     count = 1
            # else:
            #     count = 0.000001
            #     position_x = 0
            #     position_y = 0
            #     position_z = 0
            #     trans_yaw = 0
            #     trans_roll = 0
            #     trans_pitch = 0
            if id_list:
                for id in id_list:
                    markeratmap = 'aruco/marker' + str(id)
                    detectedmarker = 'aruco/detected' +str(id)
                    trans2 = tfBuffer.lookup_transform(detectedmarker,'cf1/odom',rospy.Time(0),rospy.Duration(0.1))
                    trans1 = tfBuffer.lookup_transform('map',markeratmap,rospy.Time(0),rospy.Duration(0.1))
                    trans = multiply_transforms(trans1.transform, trans2.transform)
                    queue.append(trans)
                compute_avg_transform()
                #     position_x=position_x+trans.transform.translation.x
                #     position_y=position_y+trans.transform.translation.y
                #     position_z=position_z+trans.transform.translation.z
                #     roll, pitch, yaw = euler_from_quaternion((trans.transform.rotation.x,
                #                                               trans.transform.rotation.y,
                #                                               trans.transform.rotation.z,
                #                                               trans.transform.rotation.w))
                #     trans_yaw=trans_yaw + math.degrees(yaw)
                #     trans_roll =trans_roll+math.degrees(roll)
                #     trans_pitch = trans_pitch +math.degrees(pitch)
                #     count +=1
                # position_x = position_x / count
                # position_y = position_y / count
                # position_z = position_z / count
                # trans_yaw = trans_yaw/ count
                # trans_pitch = trans_pitch/count
                # trans_roll = trans_roll/count
                # # old_trans = [position_x,position_y,position_z,trans_yaw,trans_pitch,trans_roll]
                # (orientation_x, orientation_y, orientation_z, orientation_w) = quaternion_from_euler(math.radians(trans_roll),
                #                                                                                      math.radians(trans_pitch),
                #                                                                                      math.radians(trans_yaw))
                # trans_average = TransformStamped()
                # trans_average.transform.translation.x = position_x
                # trans_average.transform.translation.y = position_y
                # trans_average.transform.translation.z = position_z
                # trans_average.transform.rotation.x = orientation_x
                # trans_average.transform.rotation.y = orientation_y
                # trans_average.transform.rotation.z = orientation_z
                # trans_average.transform.rotation.w = orientation_w
                #
                # t = TransformStamped()
                # t.transform.translation = trans_average.transform.translation
                # t.transform.rotation = trans.transform.rotation
                t = trans_average
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = 'map'
                t.child_frame_id = 'cf1/odom'
                br = tf2_ros.TransformBroadcaster()
                br.sendTransform(t)
                found = True
                print('detecting marker')



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('not detecting marker')
            if found:
                old_transform = trans_average
                old_transform.header.stamp = rospy.Time.now()
                old_transform.header.frame_id = 'map'
                old_transform.child_frame_id = 'cf1/odom'
                br = tf2_ros.TransformBroadcaster()
                br.sendTransform(old_transform)

            pass

if __name__ == "__main__":
    main()
