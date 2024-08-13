import numpy as np
from dataclasses import dataclass
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class anticollision:
    """
    The parameters for the anti-collision system
    
    Attributes:
    -----------
    VEHICLE_SPEED: float
        The speed of the vehicle (m/s)
    PDT: float
        Distance threshold for the vehicle to stop (m)
    CASE: int
        The test case of the vehicle (1 or 2). This defines the waypoints of the vehicle
    """
    SPEED = 8 * 0.278
    PDT = 0.3
    CASE = 1 # 1 or 2

class danger:
    """
    The danger level of the tracked object

    Attributes:
    -----------
    HIGH: float
        The danger level of the tracked object is high 
    MEDIUM: float
        The danger level of the tracked object is medium
    LOW: float
        The danger level of the tracked object is low
    """

    HIGH = 0.5
    MEDIUM = 5
    LOW = 10

class mode:
    """
    The mode of the system

    Attributes:
    -----------
    SIMULATION: bool
        The system is in simulation mode
    CALCULATE_NEW_POSE_FOR_OLD_BBOX: bool
        Calculate the new pose for the old bounding box
    KEEP_OLD_CLUSTERS: bool
        Keep the old clusters
    REMOVE_ENCLOSED_CLUSTERS: bool
        Remove the enclosed clusters
    OBB_MODE: str
        The mode of the oriented bounding box (2d or 3d)
    REDUCED_FREQ: bool
        The frequency of the system is reduced
    ENABLE_BODY_BBOX_ONLY: bool
        Enable the body bounding box only
    ENABLE_V2P: bool
        Enable the vehicle to pedestrian communication
    """
    LIDAR = "neuvition" # "velodyne" or "neuvition"
    SIMULATION = True
    CALCULATE_NEW_POSE_FOR_OLD_BBOX = False
    KEEP_OLD_CLUSTERS = True # Store historical clusters
    REMOVE_ENCLOSED_CLUSTERS = False
    OBB_MODE = "3d"
    REDUCED_FREQ = True
    ENABLE_BODY_BBOX_ONLY = False
    ENABLE_V2P = False


# DBSCAN and downsampling parameters
class prefiltering:
    """
    The parameters for the prefiltering of the point cloud

    Attributes:
    -----------
    VOXEL_SIZE: float
        The size of the voxel grid downsampling (m)
    MIN_X: float
        Minimum height in X axis (m)
    MAX_X: float
        Maximum height in X axis (m)
    MIN_Y: float 
        Minimum height in Y axis (m)
    MAX_Y: float
        Maximum height in Y axis (m)
    MIN_Z: float
        Minimum height in Z axis (m)
    MAX_Z: float    
        Maximum height in Z axis (Value from datasheet. Maximum height of the truck is 21m)
    """
    VOXEL_SIZE = 0.7 # Adjust the voxel size as needed
    if mode.LIDAR == "velodyne":
        MIN_X = 0.5 # Minimum height in X axis
        MAX_X = 3 # Maximum height in X axis
        MIN_Y = -1.0 # Minimum height in Y axis
        MAX_Y = 1.0 # Maximum height in Y axis
        MIN_Z = -2 # Minimum height in Z axis
        MAX_Z = 2 # Maximum height in Z axis (Value from datasheet. Maximum height of the truck is 21m)
    elif mode.LIDAR == "neuvition":
        MIN_X = 0.5
        MAX_X = 3
        MIN_Y = -1.5
        MAX_Y = 1.5
        MIN_Z = -2
        MAX_Z = 2


class clustering:
    """
    The parameters for the clustering of the point cloud

    Attributes:
    -----------
    EPSILON: float
        Maximum distance between two samples for one to be considered as in the neighborhood of the other
    MIN_SAMPLES: int
        The number of samples in a neighborhood for a point to be considered as a core point
    """

    EPSILON = 0.1  # Maximum distance between two samples for one to be considered as in the neighborhood of the other.
    MIN_SAMPLES = 10 # The number of samples in a neighborhood for a point to be considered as a core point.

class colors:
    """
    The colors of the markers

    Attributes:
    -----------
    RED: list
        The color of the marker is red
    GREEN: list
        The color of the marker is green
    BLUE: list
        The color of the marker is blue
    YELLOW: list    
        The color of the marker is yellow
    ORANGE: list    
        The color of the marker is orange
    CYAN: list  
        The color of the marker is cyan
    MAGENTA: list   
        The color of the marker is magenta  
    WHITE: list 
        The color of the marker is white  
    BLACK: list
        The color of the marker is black
    """
    RED = [1, 1, 0, 0]
    GREEN = [1, 0, 1, 0]
    BLUE = [1, 0, 0, 1]
    YELLOW = [1, 1, 1, 0]
    ORANGE = [1, 1, 0.5, 0]
    CYAN = [1, 0, 1, 1]
    MAGENTA = [1, 1, 0, 1]
    WHITE = [1, 1, 1, 1]
    BLACK = [1, 0, 0, 0]

class publish:
    """
    The publishers of the system

    Attributes:
    -----------
    filtered_pcd_pub: rospy.Publisher
        The publisher of the filtered point cloud
    path_pub: rospy.Publisher
        The publisher of the vehicle path
    bbox_marker_pub: rospy.Publisher    
        The publisher of the bounding box markers
    truck_bbox_marker_pub: rospy.Publisher
        The publisher of the truck bounding box markers
    text_marker_pub: rospy.Publisher
        The publisher of the text markers
    centroid_marker_pub: rospy.Publisher    
        The publisher of the centroid markers
    """
    filtered_pcd_pub = rospy.Publisher("/filtered_pcd", PointCloud2, queue_size=10)
    path_pub = rospy.Publisher("/vehicle_path", MarkerArray, queue_size=30)
    bbox_marker_pub = rospy.Publisher("/bbox_markers", Marker, queue_size=30)
    truck_bbox_marker_pub = rospy.Publisher("/truck_bbox_markers", Marker, queue_size=30)
    text_marker_pub = rospy.Publisher("/text_markers", Marker, queue_size=30)
    centroid_marker_pub = rospy.Publisher("/centroid_markers", Marker, queue_size=30)
    #distance_marker_pub = rospy.Publisher("/distance_markers", Marker, queue_size=30)
