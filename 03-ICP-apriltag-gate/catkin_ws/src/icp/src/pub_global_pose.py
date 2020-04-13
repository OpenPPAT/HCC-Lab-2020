#!/usr/bin/env python
# It's called 3D point correspondence problem
# A = RB + T
# B are points observed by robots
# A are points observed by global (points in global frame)
import rospy
import roslib
import tf
from geometry_msgs.msg import PoseStamped
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
from tf import transformations as tr
import math

# gp1 = np.array([0.4865, 1.3275, 0.5955])
# gp2 = np.array([0.4895, 0.251, 2.74]) 
# gp3 = np.array([0.505, -1.3155, 1.0965])
# tags = [4, 5, 6]
# A_points = {4: gp1, 5: gp2, 6: gp3}

gp1 = np.array([0.501, 1.326, 0.596])
gp2 = np.array([0.4675, 0.2515, 2.686]) 
gp3 = np.array([0.4835, -1.34, 1.092])
tags = [24, 25, 26]
A_points = {24: gp1, 25: gp2, 26: gp3}


T_GtoP = []

# def isclose(x, y, rtol=1.e-5, atol=1.e-8):
#     return abs(x-y) <= atol + rtol * abs(y)
# def euler_angles_from_rotation_matrix(R):
#     '''
#     From a paper by Gregory G. Slabaugh (undated),
#     "Computing Euler angles from a rotation matrix
#     '''
#     phi = 0.0
#     if isclose(R[2,0],-1.0):
#         theta = math.pi/2.0
#         psi = math.atan2(R[0,1],R[0,2])
#     elif isclose(R[2,0],1.0):
#         theta = -math.pi/2.0
#         psi = math.atan2(-R[0,1],-R[0,2])
#     else:
#         theta = -math.asin(R[2,0])
#         cos_theta = math.cos(theta)
#         psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
#         phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
#     return psi, theta, phi

def get_transform(p):

    p1 = np.array(p[0])
    p2 = np.array(p[1])
    p3 = np.array(p[2])

    u = p2-p1 # first col of the rotation matrix
    w = np.cross(u, (p3-p1)) # third col of the rotation matrix
    v = np.cross(w, u) # second col of the rotation matrix

    u = u/np.linalg.norm(u, ord=2)
    v = v/np.linalg.norm(v, ord=2)
    w = w/np.linalg.norm(w, ord=2)

    tr_matrix = np.array([np.append(u, 0),np.append(v, 0),np.append(w, 0),np.append(p1, 1)])
    tr_matrix = np.transpose(tr_matrix)

    return tr_matrix

# T_GtoP = tr.inverse_matrix(get_transform(np.array([gp1, gp2, gp3])))
T_GtoP = get_transform(np.array([gp1, gp2, gp3]))

class PubGlobalPose(object):
    """docstring for PubGlobalPose."""
    def __init__(self):
        super(PubGlobalPose, self).__init__()

        self.sub_at = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.apriltags_cb, queue_size=20)
        self.pub_pose = rospy.Publisher("global_pose", PoseStamped, queue_size=1)

    def apriltags_cb(self, msg):

        wanted_tags = 0
        B_points = {}
        for detection in msg.detections:
            if detection.id[0] not in tags:
                continue;
            else:
                # cuz transform from camera frame to robot frame
                p = [detection.pose.pose.pose.position.z, -detection.pose.pose.pose.position.x, -detection.pose.pose.pose.position.y]
                B_points[detection.id[0]] = p
                wanted_tags += 1


        if wanted_tags != 3:
            return
        else:
            self.cal_pose(B_points, msg.header.stamp)


    def cal_pose(self, B_points, timestamp):
        # Three orthogonal unit vector forms the transformations
        # from the origin coordination to the new coordination
        # built by these three vectors
        global T_GtoP

        rp1 = np.array(B_points[tags[0]])
        rp2 = np.array(B_points[tags[1]])
        rp3 = np.array(B_points[tags[2]])
        # T_PtoR = get_transform(np.array([rp1, rp2, rp3]))
        T_PtoR = tr.inverse_matrix(get_transform(np.array([rp1, rp2, rp3])))

        T_GtoR = np.dot(T_GtoP, T_PtoR)

        robot_tran = T_GtoR[:, 3]
        robot_qua = tr.quaternion_from_matrix(T_GtoR)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "global"
        pose_msg.pose.position.x = robot_tran[0]
        pose_msg.pose.position.y = robot_tran[1]
        pose_msg.pose.position.z = robot_tran[2]
        pose_msg.pose.orientation.x = robot_qua[0]
        pose_msg.pose.orientation.y = robot_qua[1]
        pose_msg.pose.orientation.z = robot_qua[2]
        pose_msg.pose.orientation.w = robot_qua[3]
        
        rospy.loginfo("publish pose")
        self.pub_pose.publish(pose_msg)

if __name__=="__main__":
	# Tell ROS that we're making a new node.
    rospy.init_node("global_pose_node",anonymous=False)
    print "Global Pose Main"
    node = PubGlobalPose()
    rospy.spin()
