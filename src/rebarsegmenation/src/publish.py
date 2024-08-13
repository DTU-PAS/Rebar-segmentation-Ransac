import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from defines import *
import struct

def delete_markers(pub, frame_id="world"):
    """ 
    Delete all the markers in rviz
    
    Parameters:
    -----------
    pub: rospy.Publisher
        The publisher of the markers
        
    Returns:
    --------
        None
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.action = marker.DELETEALL
    pub.publish(marker)

def delete_marker_array(pub, frame_id="world"):
    """
    Delete all the marker arrays in rviz

    Parameters:
    -----------
    pub: rospy.Publisher
        The publisher of the marker arrays
    
    Returns:
    --------
        None
    """
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.action = marker.DELETEALL
    marker_array.markers.append(marker)
    pub.publish(marker_array)

def publish_ball(coord, size, ID, ns, color, type="Sphere", orient=None, frame_id="world"):
    """ 
    Publish a ball or arrow marker
    
    Parameters:
    ----------- 
    coord: list
        The coordinate of the marker
    size: float 
        The size of the marker
    ID: int
        The id of the marker
    ns: str
        The namespace of the marker
    color: list 
        The color of the marker
    type: str, default "Sphere"
        The type of the marker
    orient: list, default None
        The orientation of the marker

    Returns:
    --------    
        None    
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    # marker.lifetime = rospy.Duration(0.2)
    marker.ns = ns
    if type == "Sphere":
        marker.type = Marker.SPHERE
    elif type == "Arrow":
        marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = color[0]
    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.id = ID
    marker.pose.position.x = coord[0]
    marker.pose.position.y = coord[1]
    if mode.OBB_MODE == "2d":
        marker.pose.position.z = 0
    elif mode.OBB_MODE == "3d":
        marker.pose.position.z = coord[2]
    if orient is None:
        marker.pose.orientation.w = 1
    else:
        marker.pose.orientation.x = orient[0]
        marker.pose.orientation.y = orient[1]
        marker.pose.orientation.z = orient[2]
        marker.pose.orientation.w = orient[3]
    publish.centroid_marker_pub.publish(marker)

def publish_box(corners, ID, ns, danger_level=None, frame_id="world"):
    """
    Publish the oriented bounding box

    Parameters:
    -----------
    corners: list
        The corners of the oriented bounding box
    ID: int
        The id of the marker
    ns: str
        The namespace of the marker
    danger_level: int, default None
        The danger level of the tracked object
        
    Returns:
    --------
        None
    """
    bbox_marker = Marker()
    bbox_marker.header.frame_id = frame_id
    bbox_marker.header.stamp = rospy.Time.now()
    bbox_marker.ns = ns #"truck_obb"
    bbox_marker.type = Marker.LINE_LIST
    bbox_marker.action = Marker.ADD
    bbox_marker.pose.orientation.w = 1.0
    bbox_marker.scale.x = 0.05  # Width of the lines
    bbox_marker.color.a = 1.0
    if danger_level == 2:
        bbox_marker.color.r = 1.0
        bbox_marker.color.g = 0.0
        bbox_marker.color.b = 0.0
    elif danger_level == 1:
        bbox_marker.color.r = 1.0
        bbox_marker.color.g = 1.0
        bbox_marker.color.b = 0.0
    elif danger_level == 0:
        bbox_marker.color.r = 0.0
        bbox_marker.color.g = 1.0
        bbox_marker.color.b = 0.0
    else:
        bbox_marker.color.r = 0.0
        bbox_marker.color.g = 0.0
        bbox_marker.color.b = 1.0
    bbox_marker.id = ID

    # Add all lines of the OBB edges
    # Add all lines of the OBB edges
    if mode.OBB_MODE == "3d":
        indices = [0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7]  # Index pairs to draw lines between
    elif mode.OBB_MODE == "2d":
        indices = [0, 1, 1, 2, 2, 3, 3, 0]
    for j in range(0, len(indices), 2):
        p1 = corners[indices[j]]
        p2 = corners[indices[j + 1]]
        bbox_marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2] if mode.OBB_MODE == "3d" else 0))
        bbox_marker.points.append(Point(x=p2[0], y=p2[1], z=p2[2] if mode.OBB_MODE == "3d" else 0))
    publish.bbox_marker_pub.publish(bbox_marker)

def publish_text(coord, ID, ns, color, scale, dist=None, frame_id="world"):
    """
    Publish the distance text marker

    Parameters:
    -----------
    coord: list
        The coordinate of the text marker
    ID: int
        The id of the marker
    ns: str
        The namespace of the marker
    color: list
        The color of the marker
    scale: float
        The scale of the marker

    Returns:
    --------
        None
    """

    text_marker = Marker()
    text_marker.header.frame_id = frame_id
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = ns #"distance_text"
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.scale.z = scale  # Adjust the size of the text as needed
    text_marker.color.a = color[0]
    text_marker.color.r = color[1]
    text_marker.color.g = color[2]
    text_marker.color.b = color[3]
    text_marker.id = ID  # Use a unique ID for each text marker
    if dist is None:
        text_marker.text = str(ID)
    else:
        text_marker.text = "ID: {} Dist: {:.2f}m".format(ID, dist)  # The text to display the distance

    text_marker.pose.position.x = coord[0]
    text_marker.pose.position.y = coord[1]
    if mode.OBB_MODE == "2d":
        text_marker.pose.position.z = 0.2  # Adjust the height above the objects as needed
    elif mode.OBB_MODE == "3d":
        text_marker.pose.position.z = coord[2] + 0.2# Adjust the height above the objects as needed

    # Publish the distance text marker
    publish.text_marker_pub.publish(text_marker)

def publish_trajectory(points, id, ns, color, frame_id="world"):
    """
    Publish the trajectory of the truck
    
    Parameters:
    -----------
    points: list
        The points of the trajectory
    id: int
        The id of the marker
    ns: str
        The namespace of the marker
    color: list
        The color of the marker

    Returns:
    -------- 
        None
    """
    marker_array = MarkerArray()
    path_list_marker = Marker()
    path_list_marker.header.stamp = rospy.Time.now()
    path_list_marker.header.frame_id = frame_id
    path_list_marker.type = Marker.LINE_STRIP
    path_list_marker.action = Marker.ADD
    path_list_marker.id = id
    path_list_marker.ns = ns

    path_list_marker.pose.orientation.w = 1.0
    path_list_marker.scale.x = 0.05  # Width of the lines
    path_list_marker.color.a = color[0]
    path_list_marker.color.r = color[1]
    path_list_marker.color.g = color[2]
    path_list_marker.color.b = color[3]
    for x, y, theta in points:
        point = Point()
        point.x = x
        point.y = y
        path_list_marker.points.append(point)
        marker_array.markers.append(path_list_marker)
    publish.path_pub.publish(marker_array)

def publish_vector(points, id, ns, color, frame_id="world"):
    """
    Publish the vector of the truck
    
    Parameters:
    -----------
    points: list
        The points of the vector
    id: int
        The id of the marker
    ns: str
        The namespace of the marker
    color: list
        The color of the marker
        
    Returns:
    --------
        None
        """
    marker_array = MarkerArray()
    path_list_marker = Marker()
    path_list_marker.header.stamp = rospy.Time.now()
    path_list_marker.header.frame_id = frame_id
    path_list_marker.type = Marker.LINE_STRIP
    path_list_marker.action = Marker.ADD
    path_list_marker.id = id
    path_list_marker.ns = ns

    path_list_marker.pose.orientation.w = 1.0
    path_list_marker.scale.x = 0.05  # Width of the lines
    path_list_marker.color.a = color[0]
    path_list_marker.color.r = color[1]
    path_list_marker.color.g = color[2]
    path_list_marker.color.b = color[3]
    for x, y, z in points:
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        path_list_marker.points.append(point)
        marker_array.markers.append(path_list_marker)
    publish.path_pub.publish(marker_array)

def publish_filtered_pcd(coord, format="pc2", frame_id="world"):
    """
    Publish the filtered point cloud
    
    Parameters:
    -----------
    pcd: PointCloud2
        The filtered point cloud
    frame_id: str, default "world"
        The frame id of the point cloud
        
    Returns:
    --------
        None
    """
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    if format == "pc2": 
        publish.filtered_pcd_pub.publish(coord)
    elif format == "numpy":
        # Not working yet

        pc2 = point_cloud2.create_cloud(header, fields, points_list)
        publish.filtered_pcd_pub.publish(pc2)