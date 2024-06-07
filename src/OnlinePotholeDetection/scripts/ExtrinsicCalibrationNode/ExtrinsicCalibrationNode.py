#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from scipy.spatial.transform import Rotation
#import open3d as o3d

import rospy
import message_filters
import sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
import tf.transformations as tf
from geometry_msgs.msg import TransformStamped


def matrix_to_quaternion(matrix):
    """
    Convert a (3, 3) rotation matrix to quaternions.

    Args:
        matrix (np.ndarray): Rotation matrix of shape (3, 3).

    Returns:
        np.ndarray: Quaternion of shape (4,).
    """
    # Pad the rotation matrix with an additional row and column of zeros
    padded_matrix = np.pad(matrix, [(0, 1), (0, 1)], mode='constant')

    # Set the bottom-right element of the padded matrix to 1
    padded_matrix[-1, -1] = 1.0

    # Compute the quaternion using quaternion_from_matrix() function
    quaternion = tf.quaternion_from_matrix(padded_matrix)

    return quaternion


def calculate_extrinsic_params(pc1, pc2, n_samples=1000):
    """
    Calculate extrinsic parameters of two 3D point clouds of different sizes.

    Args:
        pc1 (np.ndarray): First point cloud of shape (n1, 3).
        pc2 (np.ndarray): Second point cloud of shape (n2, 3).
        n_samples (int): Number of points to use for covariance matrix computation.

    Returns:
        R (np.ndarray): Rotation matrix of shape (3, 3).
        t (np.ndarray): Translation vector of shape (3, 1).
    """
    # Randomly select a subset of points from both point clouds
    # idx1 = np.random.choice(pc1.shape[0], n_samples, replace=False)
    # idx2 = np.random.choice(pc2.shape[0], n_samples, replace=False)
    pc1_sub = pc1#[idx1, :]
    pc2_sub = pc2#[idx2, :]

    # Find centroids of both point clouds
    centroid1 = np.mean(pc1_sub, axis=0, keepdims=True)
    centroid2 = np.mean(pc2_sub, axis=0, keepdims=True)

    # Center both point clouds around their centroids
    pc1_centered = pc1_sub - centroid1
    pc2_centered = pc2_sub - centroid2

    # Calculate the covariance matrix
    H = np.matmul(pc1_centered.T, pc2_centered)

    # Calculate the singular value decomposition of the covariance matrix
    U, _, Vt = np.linalg.svd(H)

    # Calculate the optimal rotation matrix
    R = np.matmul(Vt.T, U.T)

    # If the determinant of the rotation matrix is -1, flip the sign of the last column
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1

    # Calculate the translation vector
    t = np.matmul(centroid1.T - R, centroid2.T)

    return R, t


class Calibrate():
    def __init__(self):
        rospy.init_node('point_cloud_sync_node', anonymous=True)

        self.publisher = rospy.Publisher('/neuvition_cloud2', PointCloud2, queue_size=10)
        self.broadcaster = TransformBroadcaster()

        # set up the subscribers
        self.pcd_sub1 = message_filters.Subscriber('/neuvition_cloud', PointCloud2)
        self.pcd_sub2 = message_filters.Subscriber('/camera/depth/color/points', PointCloud2)

        # set up the synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pcd_sub1, self.pcd_sub2], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

    def publish_pointcloud(self, points, frame_id='world', stamp=None):
        """
        Publish a point cloud message .

        Args:
            points (numpy.ndarray): A NumPy array of shape (n_points, 3) containing the x, y, and z coordinates
                of each point in the point cloud.
            frame_id (str, optional): The coordinate frame in which the point cloud is defined. Defaults to 'world'.
            stamp (rospy.Time, optional): The time stamp of the message. If not provided, the current time will be used.
        """
        # Create a header for the message
        header = Header()
        header.stamp = stamp if stamp is not None else rospy.Time.now()
        header.frame_id = frame_id

        # Create a list of fields for the point cloud message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        # Create the point cloud message
        pointcloud_msg = point_cloud2.create_cloud(header, fields, points)

        # Publish the message 
        self.publisher.publish(pointcloud_msg)

    def publish_transformation(self, translation, rotation, parent_frame, child_frame, time):
        """
        Publishes a ROS transformation.

        Args:
            translation (np.ndarray): Translation vector of shape (3, 1).
            rotation (np.ndarray): Rotation matrix of shape (3, 3).
            parent_frame (str): Name of the parent frame.
            child_frame (str): Name of the child frame.
            time (rospy.Time): Time at which the transformation is published (default: rospy.Time.now()).
        """

        # Create a TransformStamped message
        transform_msg = TransformStamped()

        # Set the header frame ID and timestamp
        transform_msg.header.stamp = time
        transform_msg.header.frame_id = parent_frame
        transform_msg.child_frame_id = child_frame

        # Set the translation vector
        transform_msg.transform.translation.x = translation[0]
        transform_msg.transform.translation.y = translation[1]
        transform_msg.transform.translation.z = translation[2]

        # Set the rotation matrix
        rotation_quaternion = matrix_to_quaternion(rotation)
        transform_msg.transform.rotation.x = rotation_quaternion[0]
        transform_msg.transform.rotation.y = rotation_quaternion[1]
        transform_msg.transform.rotation.z = rotation_quaternion[2]
        transform_msg.transform.rotation.w = rotation_quaternion[3]

        # Publish the transformation
        self.broadcaster.sendTransform(transform_msg)

    def convert_pc_msg_to_np(self, pointcloud_msg):
        """
        Convert a ROS PointCloud2 message to a NumPy array.

        Args:
            pointcloud_msg (sensor_msgs.msg.PointCloud2): The ROS PointCloud2 message.

        Returns:
            numpy.ndarray: A NumPy array of shape (n_points, n_fields), where n_points is the
                number of points in the point cloud and n_fields is the number of fields in the
                point cloud.
        """
        # Parse the point cloud message and get the field names and data types
        fields = [(field.name, field.offset, field.datatype) for field in pointcloud_msg.fields]
        dtype_list = []
        for field in fields:
            name = field[0]
            offset = field[1]
            datatype = field[2]
            if 1 == datatype:
                dtype = np.int8
            elif 2 == datatype:
                dtype = np.uint8
            elif 3 == datatype:
                dtype = np.int16
            elif 4 == datatype:
                dtype = np.uint16
            elif 5 == datatype:
                dtype = np.int32
            elif 6 == datatype:
                dtype = np.uint32
            elif 7 == datatype:
                dtype = np.float32
            elif 8 == datatype:
                dtype = np.float64
            dtype_list.append((name, dtype, offset))

        # Convert the ROS point cloud message to a NumPy array
        point_generator = point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names=('x', 'y', 'z'))
        data = np.array(list(point_generator))
        return data

    def callback(self, pcd1, pcd2):
        """
        Synchronizes two point cloud topics and processes the received messages.
        :param pcd1: the first point cloud message.
        :param pcd2: the second point cloud message.
        """
        # process the point cloud messages here
        pcd1 = self.convert_pc_msg_to_np(pcd1)
        pcd2 = self.convert_pc_msg_to_np(pcd2)
        pcd1 = np.matmul(pcd1, Rotation.from_euler('xyz', [3.1415, 1.57079632679, -1.57079632679], degrees=False).as_dcm())
        pcd1[:, 0] = -pcd1[:, 0]
        # R, t = calculate_extrinsic_params(pcd1, pcd2, 5000)

        # trans_init = np.eye(4)
        # trans_init[:3, -1] = np.array([-0.05, -0.015, -0.065])
        # pc1 = o3d.geometry.PointCloud()
        # pc2 = o3d.geometry.PointCloud()
        # pc1.points = o3d.utility.Vector3dVector(pcd1)
        # pc2.points = o3d.utility.Vector3dVector(pcd2)

        # reg_p2p = o3d.registration.registration_icp(
        #     pc1, pc2, 0.8, trans_init,
        #     o3d.registration.TransformationEstimationPointToPoint(),
        #     o3d.registration.ICPConvergenceCriteria(max_iteration=5000))
        # print(reg_p2p)
        # print("Transformation is:")
        # print(reg_p2p.transformation)

        R = np.eye(3, dtype=np.float32)
        t = np.array([-0.05, -0.015, -0.065])
        # R = reg_p2p.transformation[:3, :3]
        # t = reg_p2p.transformation[:3, -1]
        self.publish_transformation(t, R, 'camera_link', 'camera_link2', time=rospy.Time.now())
        self.publish_pointcloud(pcd1, frame_id='camera_link2')


def main():
    while not rospy.is_shutdown():
        ec = Calibrate()
        rospy.spin()

    

if __name__ == '__main__':
    main()