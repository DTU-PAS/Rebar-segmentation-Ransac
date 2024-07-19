import rospy
import cv2
import numpy as np
import random
from skimage.morphology import skeletonize
from scipy.linalg import svd
import matplotlib.pyplot as plt
from plantcv import plantcv as pcv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from publish import *
from defines import *
import time
from sklearn.decomposition import PCA
from mpl_toolkits import mplot3d
from scipy.ndimage import convolve
# from segment_anything import SamAutomaticMaskGenerator


# import torch
# from segment_anything import sam_model_registry

# DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
# MODEL_TYPE = "vit_h"

# sam = sam_model_registry[MODEL_TYPE](checkpoint="/home/larde/Documents/GitHub/Rebar-segmentation-Ransac/sam_vit_h_4b8939.pth")
# sam.to(device=DEVICE)
# mask_generator = SamAutomaticMaskGenerator(sam)


# HEIGHT = 480
# WIDTH = 640

# HEIGHT = 480
# WIDTH = 848

HEIGHT = 720
WIDTH = 1280


def open_by_reconstruction(src, iterations = 1, ksize = 3, debug_level = 0):
        eroded = cv2.erode(src, np.ones((5,5), np.uint8), iterations=4)
        # Now we are going to iteratively regrow the eroded mask.
        # The key difference between just a simple opening is that we
        # mask the regrown everytime with the original src.
        # Thus, the dilated mask never extends beyond where it does in the original.
        this_iteration = eroded
        last_iteration = eroded
        while (True):
            this_iteration = cv2.dilate(last_iteration, np.ones((4,4), np.uint8), iterations = 1)
            # Show the image at this iteration
            if debug_level == 2:
                cv2.imshow('reconstruction - dialted', this_iteration)
                cv2.waitKey(1)
            this_iteration = this_iteration & src
            # Show the image at this iteration
            if debug_level == 2:
                cv2.imshow('reconstruction - masked', this_iteration)
                cv2.waitKey(1)
            if np.array_equal(last_iteration, this_iteration):
                # convergence!
                break
            last_iteration = this_iteration.copy()
        
        if debug_level == 1 or debug_level == 2:
            cv2.imshow('reconstruction - final', this_iteration)
            cv2.waitKey(1)
        return this_iteration

def reconstruct_skeleton(name, src, orig,  ksize = 3, iterations=15, debug_level = 0):
    this_iteration = src
    last_iteration = src
    for i in range(iterations):
        this_iteration = cv2.dilate(last_iteration, np.ones((ksize,ksize), np.uint8), iterations = 1)
        # Show the image at this iteration
        if debug_level > 1 :
            cv2.imshow(f'reconstruction {name} - dialted', this_iteration)
            cv2.waitKey(1)
        this_iteration = this_iteration & orig
        # Show the image at this iteration
        if debug_level > 1:
            cv2.imshow(f'reconstruction {name} - masked', this_iteration)
            cv2.waitKey(1)
        if np.array_equal(last_iteration, this_iteration):
            # convergence!
            break
        last_iteration = this_iteration.copy()
    
    
    skeleton, segmented_img, segment_objects = pcv.morphology.prune(skel_img=this_iteration, size=100, mask=orig)
    if debug_level > 0:
        cv2.imshow(f"reconstruction input {name} - skeleton", src)
        cv2.waitKey(1)
        cv2.imshow(f'reconstruction output {name} - pruned', skeleton)
        cv2.waitKey(1)
    
    return skeleton

def cluster_skeleton(name, skeleton, debug_level=0):
    height, width = skeleton.shape
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(skeleton, connectivity=8)
    

    cluster_img = np.zeros((height, width, 3), dtype=np.uint8)

    colors = [random_color() for _ in range(num_labels)]

    for y in range(height):
        for x in range(width):
            if skeleton[y, x] == 255:
                cluster_img[y, x] = colors[labels[y, x]]

    if debug_level:
        cv2.imshow(f'Clustered {name}', cluster_img)
        cv2.waitKey(1)
    return num_labels-1, labels, stats, centroids, cluster_img

def pixel_to_camera(u, v, Z, K_inv):
    # Normalize pixel coordinates
    pixel_coords = np.array([u, v, 1])
    camera_coords = Z * K_inv.dot(pixel_coords)
    return camera_coords

def remove_small_blobs(name, img, min_blob_size, debug_level=0):

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)

    # Create a mask to filter out small components
    large_blobs_mask = np.zeros_like(img, dtype=np.uint8)

    # Find components that are larger than the minimum blob size
    for i in range(1, num_labels):  # Start from 1 to skip the background
        if stats[i, cv2.CC_STAT_AREA] >= min_blob_size:
            large_blobs_mask[labels == i] = 255

    # Apply the mask to the input image to retain only large components
    output = cv2.bitwise_and(img, img, mask=large_blobs_mask)

    if debug_level:
        cv2.imshow(f'Input {name}', img)
        cv2.waitKey(1)
        cv2.imshow(f'Small blobs removed output {name}', output)
        cv2.waitKey(1)

    return output

def random_color():
    """
    Generate a random color.
    """
    return [random.randint(0, 255) for _ in range(3)]

def fit_line_to_points(points):
    """
    Fits a line to a set of 3D points using least squares.
    
    Parameters:
    points (numpy.ndarray): An Nx3 array of points in 3D space.
    
    Returns:
    tuple: A tuple containing:
           - point_on_line (numpy.ndarray): A point on the fitted line.
           - direction_vector (numpy.ndarray): The direction vector of the fitted line.
    """
    # Calculate the centroid of the points
    centroid = np.mean(points, axis=0)
    
    # Subtract the centroid from the points to translate the points
    translated_points = points - centroid
    
    # Perform Singular Value Decomposition (SVD) on the translated points
    _, _, vh = svd(translated_points)
    
    # The direction vector of the fitted line is the first right singular vector
    direction_vector = vh[0]
    
    return centroid, direction_vector

def split_horizontal_and_vertical(skeleton, debug_level=0):
    height, width = skeleton.shape
    pruned_vertical, pruned_horizontal = skeleton.copy(), skeleton.copy()
    for y in range(height):
            for x in range(width):
                if skeleton[y, x] == 255:
                    # check next lines in the vertical direction
                    for i in range(1, 10):
                        if y+i < height:
                            if skeleton[y+i, x] == 255:
                                pruned_horizontal[y, x] = 0
                                break
                        if y-i >= 0:
                            if skeleton[y-i, x] == 255:
                                pruned_horizontal[y, x] = 0
                                break
                    # check next lines in the horizontal direction
                    for i in range(1, 10):
                        if x+i < width:
                            if skeleton[y, x+i] == 255:
                                pruned_vertical[y, x] = 0
                                break
                        if x-i >= 0:
                            if skeleton[y, x-i] == 255:
                                pruned_vertical[y, x] = 0
                                break
    if debug_level:
        print("Vertical Shape", pruned_vertical.shape)

        cv2.imshow('pruned_vertical', pruned_vertical)
        cv2.waitKey(1)

        print("Horizontal Shape", pruned_horizontal.shape)

        cv2.imshow('pruned_horizontal', pruned_horizontal)
        cv2.waitKey(1)

    return pruned_vertical, pruned_horizontal

def rotate_image(name, image, angle, center=None, scale=1.0, debug_level=0):
    (h, w) = image.shape[:2]
    if center is None:
        center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(image, M, (w, h))

    thrsh = cv2.threshold(rotated, 0, 255, cv2.THRESH_BINARY)[1]

    skeleton = skeletonize(thrsh, method='lee').astype(np.uint8)

    if debug_level > 0:
        cv2.imshow(f"Original {name}", image)
        cv2.waitKey(1)
        cv2.imshow(f'rotated {name}', skeleton)
        cv2.waitKey(1)
    return skeleton

def find_rotation(image, debug_level=0):
    # return vertical_angle, horizontal_angle
    # find the angle of the skeleton
    # Detect lines using Hough Line Transform
    lines = cv2.HoughLinesP(image, 1, np.pi / 180, threshold=50, minLineLength=10, maxLineGap=10)
    
    # Classify lines based on their angle


    lines_array = np.array([line[0] for line in lines]).reshape(-1, 4)

    # Extract coordinates
    x1 = lines_array[:, 0]
    y1 = lines_array[:, 1]
    x2 = lines_array[:, 2]
    y2 = lines_array[:, 3]

    # Compute the angles in degrees
    angles = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

    # Determine horizontal-like and vertical-like angles
    horizontal_mask = (-45 <= angles) & (angles <= 45)
    vertical_mask = (angles > 45) & (angles <= 135) | (angles < -45) & (angles >= -135)

    # Sum and count the angles using masks
    horizontal_angles = angles[horizontal_mask]
    vertical_angles = angles[vertical_mask]

    # Compute the averages
    if horizontal_angles.size > 0:
        average_horizontal_angle = np.mean(horizontal_angles)
    else:
        average_horizontal_angle = 0  # Handle no horizontal lines case

    if vertical_angles.size > 0:
        average_vertical_angle = np.mean(vertical_angles)
    else:
        average_vertical_angle = 0  # Handle no vertical lines case
    
    if debug_level > 0:
        print("Vertical angle:", average_vertical_angle)
        print("Horizontal angle:", average_horizontal_angle)

    return average_vertical_angle, average_horizontal_angle

def plot_camera_coordinates(camera_cluster_coordinates, debug_level=0):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    ax.set_xlim([-1.2, 1.2])
    ax.set_ylim([-0.3, 0.3])
    ax.set_zlim([0, 0.7])
        

    for i, coord in enumerate(camera_cluster_coordinates):
        coord = np.array(coord)
        if coord.size == 0:
            print(f"No white pixels found for rebar {i}")
            continue

        X = coord[:, 0]
        Y = coord[:, 1]
        Z = coord[:, 2]

        # # TODO This should not be hardcoded. My first idea is improving the ransac segmentation. 
        # mask = X #<= 0.8

        # # # Filter camera coordinates using the mask
        # filtered_camera_coordinates = coord[mask]

        # # # If you want to extract the filtered X, Y, Z separately
        # filtered_X = X[mask]
        # filtered_Y = Y[mask]
        # filtered_Z = Z[mask]

        # ax.scatter(filtered_X, filtered_Y, filtered_Z, c=np.random.rand(3,), marker='o', label='%s Rebar' % i)
        ax.scatter(X, Y, Z, c=np.random.rand(3,), marker='o', label='%s Rebar' % i)
        i += 1


        # Show plot
    plt.legend()
    plt.show()

def find_area_of_interest(name, labels, num_labels, gray_orig, img, debug_level=0):

    closest_pixels = []
    bboxs = []

    # show the input labels
    if debug_level > 0:
        label_image = np.zeros_like(gray_orig)
        for i in range(1, num_labels+1):
            label_image[labels == i] = 255
        cv2.imshow(f'{name} label', label_image)
        cv2.waitKey(1)
        

    # Find the closest pixel between the vertical clusters
    for i in range(1, num_labels+1):
        for j in range(i+1, num_labels+1):
            cluster1 = np.column_stack(np.where(labels == i))
            cluster2 = np.column_stack(np.where(labels == j))
            min_distance = 1000
            for (u1, v1) in cluster1:
                for (u2, v2) in cluster2:
                    distance = np.sqrt((v1-v2)**2 + (u1-u2)**2)
                    if distance < min_distance:
                        min_distance = distance
                        closest_pixel = (v1, u1), (v2, u2)
            if min_distance < 100:
                # Visualize the damaged area in the original image 
                # draw a bounding box around the damaged area
                # make the bounding box bigger by 10 pixels in each direction
                x1, y1 = closest_pixel[0]
                x2, y2 = closest_pixel[1]
                x1, y1 = x1-10, y1-10
                x2, y2 = x2+10, y2+10
                if x1 < 0:
                    x1 = 0
                if y1 < 0:
                    y1 = 0
                if x2 > WIDTH-1:
                    x2 = WIDTH-1
                if y2 > HEIGHT-1:
                    y2 = HEIGHT-1

                # Extract the damaged area from the original image but keep the size of the image

                damaged_area = cluster_img = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
                damaged_area[y1:y2, x1:x2] = gray_orig[y1:y2, x1:x2]

                # cluster connected pixels
                # Find connected components
                num, lab, _stats, _centroids = cv2.connectedComponentsWithStats(damaged_area, connectivity=8)
                #num -= 1

                # Show clusters with different colors
                cluster_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

                colors = [random_color() for _ in range(num)]
                for y in range(HEIGHT):
                    for x in range(WIDTH):
                        if damaged_area[y, x] == 255:
                            cluster_img[y, x] = colors[lab[y, x]]
                
                if debug_level > 0:
                    cv2.imshow(f'Clustered damaged area {name}', cluster_img)
                    cv2.waitKey(1)

                for i in range(1, num+1):
                    for j in range(i+1, num+1):
                        cluster1 = np.column_stack(np.where(lab == i))
                        cluster2 = np.column_stack(np.where(lab == j))
                        min_distance = 1000
                        closest_pixel_pair = None
                        for (u1, v1) in cluster1:
                            for (u2, v2) in cluster2:
                                distance = np.sqrt((v1-v2)**2 + (u1-u2)**2)
                                if distance < min_distance:
                                    min_distance = distance
                                    closest_pixel_pair = (v1, u1), (v2, u2)
                        if closest_pixel_pair is not None:  # Check if closest_pixel_pair was updated
                            closest_pixels.append(closest_pixel_pair)
                        bboxs.append(((x1, y1), (x2, y2)))

    return closest_pixels, bboxs

image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

class ImageSubscriber:
    def __init__(self):
        self.br = CvBridge()
        # Subscribe to the depth and color image topics
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.label_sub = rospy.Subscriber('/label', Image, self.label_callback)
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        
        
        self.depth_image = None
        self.label_image = None
        self.rgb_image = None

        self.count = 0  

    def rgb_callback(self, data):
        try:
            self.rgb_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
            # cv2.imshow('RGB', self.rgb_image)
            # cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error converting color image: {e}")

    def depth_callback(self, data):
        try:
            # Convert depth image to numpy array
            dtype = np.dtype('uint16')  # Change this according to the depth image type
            depth_image = np.frombuffer(data.data, dtype=dtype).reshape(data.height, data.width)
            
            # calcualted_depth = depth_image[150, 98] * 0.001
            # if calcualted_depth >= 0.0001:
            #     rospy.loginfo(f"Received depth image: {calcualted_depth}")
            self.depth_image = depth_image
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def label_callback(self, data):
        #try:
        self.label_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # self.image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        start_time = time.time()
        self.findRebars()
        #rospy.loginfo(f"Time taken: {time.time() - start_time}")
        #rospy.loginfo("Received color image")
        # except Exception as e:
        #     rospy.logerr(f"Error converting color image: {e}")

    def findRebars(self):
        global image
        if self.count == 0:
            image = self.label_image
        elif self.count <= 1:
            # OR the images to get the final image
            image = cv2.bitwise_or(self.label_image, image)

        elif self.count > 1:
            self.count = 0
            gray_orig = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            gray_orig = cv2.dilate(gray_orig, np.ones((5,5), np.uint8), iterations=1)
            gray_orig = cv2.erode(gray_orig, np.ones((5,5), np.uint8), iterations=1)

            # cluster connected pixels
            # Find connected components
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray_orig, connectivity=8)

            # Remove clusters with less than 100 pixels
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] < 350:
                    gray_orig[labels == i] = 0

            # If the center of mass is on black background remove the cluster. 
            # The rebar is shaped in a way that the center of mass is in the middle of the rectangle which is background.
            for i in range(1, num_labels):
                x, y = int(centroids[i][0]), int(centroids[i][1])
                if gray_orig[y, x] == 255:
                    gray_orig[labels == i] = 0

            
            #gray_orig = cv2.resize(gray_orig, self.rgb_image.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
            cv2.imshow('final', gray_orig)
            cv2.waitKey(1)

            cv2.imshow('rgb', self.rgb_image)
            cv2.waitKey(1)

            # only keep the rgb pixels that are white in the gray image
            rgb_img = cv2.bitwise_and(self.rgb_image, self.rgb_image, mask=gray_orig)
            cv2.imshow('rgb_img', rgb_img)
            cv2.waitKey(1)

            
            image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.count += 1
        
       
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    img_sub = ImageSubscriber()
    img_sub.run()

