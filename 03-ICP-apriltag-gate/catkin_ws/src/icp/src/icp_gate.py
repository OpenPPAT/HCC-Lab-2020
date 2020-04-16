#! /usr/bin/env python

import rospy
import tf
import numpy as np
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection


class ICP(object):
    def __init__(self):
        self.gate_poses = np.array([[0.501, 1.326, 0.596],      # tag 4
                                    [0.4675, 0.2515, 2.686],    # tag 5
                                    [0.4835, -1.34, 1.092]])    # tag 6

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # base_footprint 2 camera_color_optical_frame
        trans_c = [0.506, 0.033, 0.425]
        rot_c = [-0.430, 0.430, -0.561, 0.561]
        self.camera2foot = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans_c), tf.transformations.quaternion_matrix(rot_c))

        sub_detection = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.cb_detection, queue_size=1)

    def centroid(self, points):
        x = np.sum(points[0, :])/3
        y = np.sum(points[1, :])/3
        z = np.sum(points[2, :])/3
        return np.array([x, y, z])

    def de_mean(self, points, centroid):
        return (points-centroid).T

    def cb_detection(self, msg):
        # check there is more than 3 detections
        # sort the detections by id
        # check detection id 4 5 6 exist

        if len(msg.detections) >= 3:
            detections = sorted(msg.detections, key=lambda s: s.id[0])
            id_check = 4
            for i in range(3):
                if i != detections[i].id[0]-4:
                    print('missing gate detection')
                    return
                else:
                    id_check += 1

            # calculate tags position relative to car
            tag_poses = []
            for i in range(3):
                p = detections[i].pose.pose.pose.position
                t = detections[i].pose.pose.pose.orientation
                pos = [p.x, p.y, p.z]
                tran = [t.x, t.y, t.z, t.w]

                tag_tf = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(pos), tf.transformations.quaternion_matrix(tran))
                tag_position = np.matmul(self.camera2foot, tag_tf)[0:3, 3]
                tag_poses.append(tag_position)
            tag_poses = np.array(tag_poses)

            # calculate centroid
            c_tag = self.centroid(tag_poses)
            c_gate = self.centroid(self.gate_poses)

            # de mean
            m_tag = self.de_mean(tag_poses, c_tag)
            m_gate = self.de_mean(self.gate_poses, c_gate)

            # calculate correlation matrix
            H = np.matmul(m_tag, m_gate.T)

            # SVD decompose
            U, D, V = np.linalg.svd(H)

            # get Rotation
            rotation = np.matmul(U, V)
            translation = c_gate - np.matmul(rotation, c_tag)

            trans0 = np.hstack((rotation[0], translation[0]))
            trans1 = np.hstack((rotation[1], translation[1]))
            trans2 = np.hstack((rotation[2], translation[2]))
            trans3 = np.zeros(4)
            trans3[3] = 1
            trans = np.vstack((trans0, trans1, trans2, trans3))

            quat = tf.transformations.quaternion_from_matrix(trans)
            self.broadcaster.sendTransform(
                translation, quat, rospy.Time.now(), "global", "map")

            print(quat)


if __name__ == "__main__":
    rospy.init_node('icp_gate')
    icp = ICP()
    rospy.spin()
