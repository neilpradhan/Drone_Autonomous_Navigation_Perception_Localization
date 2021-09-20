#!/usr/bin/env python

# E:\backup_27august_19\dd2419_ws\src\perception\scripts

import sys
from collections import defaultdict
import threading
import numpy as np
import rospy
import cv2
import math
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import StaticTransformBroadcaster
import tf2_ros
import message_filters
from PIL import Image as pilImage
import signal
import time



class TLightNode(object):
    def __init__(self, get_model_callback, model_callback):
        rospy.init_node('tlight_model')
        self.model = get_model_callback()
        self.get_model = get_model_callback
        self.predict = model_callback
        self.bridge = CvBridge()
        self.boxes = None
        self.img =  None
        self.img_out = None
        self.image_lock = threading.RLock()
      
        self.poseFile = open('detections.txt', 'a') 
        self.crop_img = None
        self.pose = None

        #list of all detections
        self.L = [] #tranfsorms
        self.allDetections = []

        print('***in tlight_model***')


        self.sub = rospy.Subscriber('/camera/image_decompressed', Image, self.updateImage, queue_size=1)
        self.subPose = rospy.Subscriber('/cf1/pose', PoseStamped, self.updatePose, queue_size=1)

        self.tf_buf   = tf2_ros.Buffer()
        self.tf_lstn  = tf2_ros.TransformListener(self.tf_buf)

        self.pub = rospy.Publisher('/out_image', Image, queue_size=1)
        self.pubCropImage = rospy.Publisher('/crop', Image, queue_size=1)
        
        signal.signal(signal.SIGINT, self.sigint_handler)

        #print('***done listening***')
        rospy.Timer(rospy.Duration(0.04), self.callbackImage)
        

    def updatePose(self, pose):
        self.pose = pose


    def updateImage( self, img):
        print("Looking for objects")

        arr = self.bridge.imgmsg_to_cv2(img,"bgr8")# converting image to ros 
        
        font = cv2.FONT_HERSHEY_COMPLEX

        #shape of object in 3d space
        sign_width=0.2          
        sign_height=0.2

        #corners in 3d space
        corners_object3d = np.array([
                                [0, 0, 0],
                                [sign_width, 0, 0],
                                [sign_width, sign_height, 0],
                                [0, sign_height, 0]])

        #camera params
        mtx= np.array([[231.250001, 0.000000, 320.519378],
              [ 0.000000, 231.065552, 240.631482],
              [ 0.000000, 0.000000, 1.000000]])

        dist= np.array([0.061687, -0.049761, -0.008166, 0.004284, 0.000000])


        if self.image_lock.acquire(True):
            self.img = arr
            if self.model is None:
                self.model = self.get_model()
            self.img_out, self.boxes = self.predict(self.model, self.img)
            self.img_out = np.asarray(self.img_out[0,:,:,:])
            
            pad = 7

            canny = cv2.Canny(self.img_out,100,200,3)

            #480 * 640 image dimension
            for box in self.boxes:
                
                #bgr
                #draw boxes
                cv2.rectangle(self.img_out,(box['topleft']['x'], 
                                            box['topleft']['y']), 
                                            (box['bottomright']['x'], 
                                            box['bottomright']['y']), 
                                            (207,161,146), 3)
                #draw label
                cv2.putText(self.img_out, box['label'], 
                           (box['topleft']['x'], 
                           box['topleft']['y'] - 12), 0, 0.6, (255,0,0) ,6//3)
          
               
#POSE ESTIMATION ###############################################################################################################33               

                #clockwise from top left top right  bottom right and finally bottom left
                c1=np.array([box['topleft']['x'],box['topleft']['y']],dtype='f')
                c2=np.array([box['bottomright']['x'],box['topleft']['y']],dtype='f')
                c3=np.array([box['bottomright']['x'],box['bottomright']['y']],dtype='f')
                c4=np.array([box['topleft']['x'],box['bottomright']['y']],dtype='f')
                
                #corneres in image plane
                corners=np.array([[c1],[c2],[c3],[c4]])

                #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
                
                #translation and rotation btw camera and world co-ordinates
                _,rvecs, tvecs = cv2.solvePnP(corners_object3d,corners, mtx, dist) 
                rotMatrix,_ = cv2.Rodrigues(rvecs)
                
                print("rot ",rotMatrix.shape)
                print("trans ",tvecs.shape) 

                convMatrix = np.hstack((rotMatrix,tvecs))                
                #print("conv ",convMatrix.shape)

                #center in 3d space
                #center3d = np.zeros((4,1))   
                center3d = np.asarray([[sign_width/2], 
                            [sign_height/2],
                            [0],
                            [1]])

                #print("center 3d ", center3d.shape)

                #final object in world w.r.t camera frame
                centerWorld = np.matmul(convMatrix,center3d)
                #print("center tranformed ", centerWorld.shape)
                print("wrt camera", np.ravel(centerWorld))
                #center in camera frame
                centerCamera = PoseStamped()
                centerCamera.header.stamp = rospy.Time.now()
                centerCamera.header.frame_id = "camera_link"
                centerCamera.pose.position.x = centerWorld[0]
                centerCamera.pose.position.y = centerWorld[1]
                centerCamera.pose.position.z = centerWorld[2]

                # quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
                # centerPose.pose.orientation.x = quaternion[0]
                # centerPose.pose.orientation.y = quaternion[1]
                # centerPose.pose.orientation.z = quaternion[2]
                # centerPose.pose.orientation.w = quaternion[3]                
                        
                if not self.tf_buf.can_transform(centerCamera.header.frame_id, 'map', centerCamera.header.stamp):
                    rospy.logwarn_throttle(5.0, 'No transform from %s to map' % centerCamera.header.frame_id)
                return

                #tranforming center in camera frame ro map frame
                centerMap = self.tf_buf.transform(centerCamera, 'map')
                print("in map", np.ravel(centerWorld))
                detection =  np.hstack((centerWorld,box['label']))                
                print("in map", np.ravel(detection))
                print(detection.shape)
                self.allDetections.append(centerMap)
                
                trans = TransformStamped()
                trans.header.frame_id = 'map'
                trans.child_frame_id = box['label']
             
                trans.transform.translation.x = centerMap.pose.position.x
                trans.transform.translation.y = centerMap.pose.position.y
                trans.transform.translation.z = centerMap.pose.position.z
                trans.transform.rotation.x = centerMap.pose.orientation.x
                trans.transform.rotation.y = centerMap.pose.orientation.y
                trans.transform.rotation.z = centerMap.pose.orientation.z
                trans.transform.rotation.w = centerMap.pose.orientation.W

                self.L.append(trans)
                

            stb = StaticTransformBroadcaster()
            stb.sendTransform(self.L)
            for item in self.L:
                self.poseFile.write("%s\n" % item)
#POSE ESTIMATION ###############################################################################################################33               
                        
    
            self.image_lock.release()
            


    def sigint_handler(self, signum, frame):
        print 'Stop pressing the CTRL+C!'
        exit()
 

    def callbackImage(self, event):
        if self.img_out is None:
            return   
