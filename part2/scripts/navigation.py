#!/usr/bin/env python
import json
import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.pyplot as plt
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import numpy as np
import tf
import rospy
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position

from grid import *
from action_a_star import *
idx = 0
move = 0
wait = 1
stop_threshold =20
dist_threshold = 0.1
NPOINTS = 8
callback_data = None
odom = None

 # is of opsition type
rx1=[]
ry1=[]
yaw_degrees=[]
rospy.init_node('planner')
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def pose_callback(msg):
#indexing every position
# the msg will be of type Posestamped
    global odom
    global idx
    global rx1,ry1,yaw_degrees
    callback_data = msg
    global stamp
    if pose_callback.state == move:
        # Compute distance between current pose and goal
        #storing in 2 variablestf_buf
        pos = callback_data.pose.position
        orient = callback_data.pose.orientation
        stamp = callback_data.header.stamp
        if len(rx1)>0:
            goal_map_to_odom = onepointpublish(rx1[idx],ry1[idx],yaw_degrees[idx])
            d =np.array([0,0])
            d = np.array([goal_map_to_odom.x-pos.x,goal_map_to_odom.y-pos.y])
            print("d",d)
            # print("pos.x",pos.x)
            distance = np.linalg.norm(d)
            print("distance",distance)
            print("idx",idx)

            if(distance < dist_threshold):
                d =np.array([0,0])
                distance = 0.0
                print("reached at index",idx)
                pose_callback.state = wait

    elif pose_callback.state == wait:
        # Wait here for some iterations, counter here which will keep counting timpes
        pose_callback.count = pose_callback.count + 1
        if (pose_callback.count > stop_threshold):
            #We've waited enough, start moving again to next point
            print("next goal")
            pose_callback.state = move
            pose_callback.count = 0

            idx = idx + 1
            print("idx inside nextgoal",idx)
            idx = idx % NPOINTS




def onepointpublish(x,y,yaw1):
    global odom
    rate = rospy.Rate(10)
    goal = PoseStamped()
    goal.header.stamp = stamp
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.4
    if not tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):
        rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')
    if goal_odom:
        #print(goal_odom.header.stamp)
        cmd = Position()

        cmd.header.stamp = rospy.Time.now()
        cmd.x = goal_odom.pose.position.x
        cmd.y = goal_odom.pose.position.y
        cmd.z = goal_odom.pose.position.z
        cmd.yaw = yaw1
        odom = cmd
        return odom









pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)


pose_callback.state = move
pose_callback.count = 0


def planner(sx,sy,gx,gy,grid_size,robot_size):

    global rx1
    global ry1
    global yaw_degrees

    grid_size = 1.0  # [m]
    robot_size = 0.0 # [m]

    print("gpoint",gpoint)
    print("gangle",gangle)

    # if show_animation:  # pragma: no cover
    #     plt.plot(ox, oy, ".k")
    #     plt.plot(sx, sy, "xr")
    #     plt.plot(gx, gy, "xb")
    #     plt.grid(True)
    #     plt.axis("equal")
    # for i in range(int(len(gangle))):
    #     plt.plot(gpoint[i][0],gpoint[i][1],"bo")
    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
    rx1 = rx1 + rx[::-1]
    ry1 = ry1 + ry[::-1]

    print("rx",rx)
    print("ry",ry)
    print("rx1",rx1)
    print("ry1",ry1)
    # if show_animation:  # pragma: no cover
    #     plt.plot(rx, ry, "-r")
    #     plt.show()


def defining_points(gpoint,idx,heading):
    
    if heading == 0.0: 
        sx = gpoint[idx][0]-division  # [m]
        sy = gpoint[idx][1]  # [m]
        gx = gpoint[idx][0]+division  # [m]
        gy = gpoint[idx][1] # [m]


    if heading == 180.0: 
        sx = gpoint[idx][0]+division  # [m]
        sy = gpoint[idx][1]  # [m]
        gx = gpoint[idx][0]-division  # [m]
        gy = gpoint[idx][1] # [m]


    if heading== 90.0 or heading == 270.0 :
        sx = gpoint[idx][0]  # [m]
        sy = gpoint[idx][1]-division  # [m]
        gx = gpoint[idx][0]  # [m]
        gy = gpoint[idx][1]+division # [m]

    if heading== -90.0 :
        sx = gpoint[idx][0]  # [m]
        sy = gpoint[idx][1]-division  # [m]
        gx = gpoint[idx][0]  # [m]
        gy = gpoint[idx][1]+division # [m]



    if heading == 135.0:
        sx = gpoint[idx][0]+division  # [m]
        sy = gpoint[idx][1]-division  # [m]
        gx = gpoint[idx][0]-division  # [m]
        gy = gpoint[idx][1]+division # [m]       

    if heading == 45.0:
        sx = gpoint[idx][0]-division  # [m]
        sy = gpoint[idx][1]-division  # [m]
        gx = gpoint[idx][0]+division  # [m]
        gy = gpoint[idx][1]+division # [m]       
    return sx,sy,gx,gy





def main():
    global rx1
    global ry1
    global yaw_degrees




    # first way to solve the problem little hardcoded and risky
    # sx = gpoint[0][0]-1  # [m]
    # sy = gpoint[0][1]  # [m]
    # gx = gpoint[0][0]  # [m]
    # gy = gpoint[0][1] # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)
    
    # sx = gpoint[0][0]  # [m]
    # sy = gpoint[0][1]  # [m]
    # gx = gpoint[0][0]+1  # [m]
    # gy = gpoint[0][1] # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)

    # sx = gpoint[0][0]+1  # [m]
    # sy = gpoint[0][1]  # [m]
    # gx = gpoint[1][0]  # [m]
    # gy = gpoint[1][1] # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)



    # sx = gpoint[1][0]  # [m]
    # sy = gpoint[1][1]  # [m]
    # gx = gpoint[1][0]  # [m]
    # gy = gpoint[1][1]+1 # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)

    # sx = gpoint[1][0]  # [m]
    # sy = gpoint[1][1]+1  # [m]
    # gx = gpoint[2][0]  # [m]
    # gy = gpoint[2][1] # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)


    # sx = gpoint[2][0]  # [m]
    # sy = gpoint[2][1]  # [m]
    # gx = gpoint[2][0]-1  # [m]
    # gy = gpoint[2][1] # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)    
    
    # sx = gpoint[2][0]-1  # [m]
    # sy = gpoint[2][1]  # [m]
    # gx = gpoint[3][0]  # [m]
    # gy = gpoint[3][1] # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)

    # sx = gpoint[3][0]  # [m]
    # sy = gpoint[3][1]  # [m]
    # gx = gpoint[3][0]  # [m]
    # gy = gpoint[3][1]-1 # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 0.0 # [m]
    # planner(sx,sy,gx,gy,grid_size,robot_size)
    # rx1 =           [0.0]
    # ry1=            [0.0]
    # yaw_degrees =   [0.0]






    grid_size = 1.0  # [m]
    robot_size = 0.0 # [m]

    #second way to solve the problem fully autonomous
    sx_0,sy_0,gx_0,gy_0 = defining_points(gpoint,0,gangle[0])
    sx1,sy1,gx1,gy1 = defining_points(gpoint,1,gangle[1])
    sx2,sy2,gx2,gy2 = defining_points(gpoint,2,gangle[2])
    sx3,sy3,gx3,gy3 = defining_points(gpoint,3,gangle[3])
    # sx4,sy4,gx4,gy4 = defining_points(gpoint,4,gangle[4])
    # sx5,sy5,gx5,gy5 = defining_points(gpoint,5,gangle[5])
    # sx6,sy6,gx6,gy6 = defining_points(gpoint,6,gangle[6])
    # sx7,sy7,gx7,gy7 = defining_points(gpoint,7,gangle[7])
              
    planner(sx_0,sy_0,gx_0,gy_0,  grid_size,robot_size)
    planner(gx_0,gy_0,sx1,sy1,    grid_size,robot_size)    
    planner(sx1,sy1,gx1,gy1,      grid_size,robot_size)
    planner(gx1,gy1,sx2,sy2,      grid_size,robot_size)
    planner(sx2,sy2,gx2,gy2,      grid_size,robot_size)    
    planner(gx2,gy2,sx3,sy3,      grid_size,robot_size)
    planner(sx3,sy3,gx3,gy3,      grid_size,robot_size)     
    # planner(gx3,gy3,sx4,sy4,      grid_size,robot_size)    
    # planner(sx4,sy4,gx4,gy4,      grid_size,robot_size)    
    # planner(gx4,gy4,sx5,sy5,      grid_size,robot_size)
    # planner(sx5,sy5,gx5,gy5,      grid_size,robot_size)
    # planner(gx5,gy5,sx6,sy6,      grid_size,robot_size)
    # planner(sx6,sy6,gx6,gy6,      grid_size,robot_size)
    # planner(gx6,gy6,sx7,sy7,      grid_size,robot_size)
    # planner(sx7,sy7,gx7,gy7,      grid_size,robot_size)

    plt.figure('final_path')
    plt.plot(ox, oy, ".k")


    #print(len(rx1))
    plt.plot(rx1,ry1,"xr")
    for i in range(int(len(gangle))):
        plt.plot(gpoint[i][0],gpoint[i][1],"bo")
    plt.show()
    plt.axis("equal")

if __name__ == '__main__':
    main()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if odom:
            print(odom)
            #pub_cmd.publish(odom)
        rate.sleep()
