import rospy
import math
import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from aruco_msgs.msg import MarkerArray
from crazyflie_gazebo.msg import Position

#subscribed to aruco/markers topic and data comming from that topic

def callback_marker(data):
    br = tf2_ros.TransformBroadcaster()

    # Transformation from camera to base link static
    baseCam =TransformStamped()
    baseCam.header.stamp = rospy.Time.now()
    baseCam.child_frame_id = "cf1/camera_link"
    baseCam.header.frame_id = "cf1/base_link"

    # Translation
    baseCam.transform.translation.x = 0.01 
    baseCam.transform.translation.y = 0 
    baseCam.transform.translation.z = 0.02
    
    # Rotation
    (baseCam.transform.rotation.x,
     baseCam.transform.rotation.y,
     baseCam.transform.rotation.z,
     baseCam.transform.rotation.w) = quaternion_from_euler(math.radians(90),
                                                                  math.radians(180),
                                                                  math.radians(90)
                                                                  )


    br.sendTransform(baseCam) #transform from camera to base link

    if __name__ == '__main__':
    rospy.init_node('basetocam')
    rate = rospy.Rate(10)  # Hz
    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)#tf listener
    marker_pub= rospy.Publisher("/trans_det", Position, queue_size=20)
    marker_sub = rospy.Subscriber("aruco/markers", MarkerArray, callback_marker)


    rospy.spin()
