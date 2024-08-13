import rospy
import cv2
import numpy as np
import random
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

# HEIGHT = 480
# WIDTH = 640

HEIGHT = 480
WIDTH = 848

# HEIGHT = 720
# WIDTH = 1280

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

def split_horizontal_and_vertical(skeleton, left_right_num=10, debug_level=0):
    height, width = skeleton.shape
    pruned_vertical, pruned_horizontal = skeleton.copy(), skeleton.copy()
    for y in range(height):
            for x in range(width):
                if skeleton[y, x] == 255:
                    # check next lines in the vertical direction
                    for i in range(1, left_right_num):
                        if y+i < height:
                            if skeleton[y+i, x] == 255:
                                pruned_horizontal[y, x] = 0
                                break
                        if y-i >= 0:
                            if skeleton[y-i, x] == 255:
                                pruned_horizontal[y, x] = 0
                                break
                    # check next lines in the horizontal direction
                    for i in range(1, left_right_num):
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

    # height, width = skeleton.shape
    # pruned_vertical = skeleton.copy()
    # pruned_horizontal = skeleton.copy()

    # # Create a shifted skeleton to detect horizontal lines
    # horizontal_shifted = np.zeros_like(skeleton)
    # for i in range(1, 10):
    #     horizontal_shifted[:, :-i] += skeleton[:, i:]
    #     horizontal_shifted[:, i:] += skeleton[:, :-i]

    # # Create a shifted skeleton to detect vertical lines
    # vertical_shifted = np.zeros_like(skeleton)
    # for i in range(1, 10):
    #     vertical_shifted[:-i, :] += skeleton[i:, :]
    #     vertical_shifted[i:, :] += skeleton[:-i, :]

    # # Prune horizontal lines
    # pruned_horizontal[horizontal_shifted > 0] = 0

    # # Prune vertical lines
    # pruned_vertical[vertical_shifted > 0] = 0

    # if debug_level:
    #     print("Vertical Shape", pruned_vertical.shape)

    #     cv2.imshow('pruned_vertical', pruned_vertical)
    #     cv2.waitKey(1)

    #     print("Horizontal Shape", pruned_horizontal.shape)

    #     cv2.imshow('pruned_horizontal', pruned_horizontal)
    #     cv2.waitKey(1)

    # return pruned_vertical, pruned_horizontal

def rotate_image(name, image, angle, center=None, scale=1.0, debug_level=0):
    (h, w) = image.shape[:2]
    if center is None:
        center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(image, M, (w, h))

    if debug_level > 0:
        cv2.imshow(f"Original {name}", image)
        cv2.waitKey(1)
        cv2.imshow(f'rotated {name}', rotated)
        cv2.waitKey(1)
    return rotated

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
        # self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        
        
        self.depth_image = None
        self.label_image = None
        # self.rgb_image = None

        self.count = 0  

    # def rgb_callback(self, data):
    #     try:
    #         self.rgb_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    #         # cv2.imshow('RGB', self.rgb_image)
    #         # cv2.waitKey(1)
    #     except Exception as e:
    #         rospy.logerr(f"Error converting color image: {e}")

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
            # cv2.imshow('Starting', image)
            # cv2.waitKey(1)
            self.count = 0
            # cv2.imshow('final', image)
            # cv2.waitKey(1)
            gray_orig = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            #gray_orig = cv2.dilate(gray_orig, np.ones((3,3), np.uint8), iterations=1)
            #gray_orig = cv2.erode(gray_orig, np.ones((3,3), np.uint8), iterations=1)

            # cluster connected pixels
            # Find connected components
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray_orig, connectivity=8)

            # Remove clusters with less than 100 pixels
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] < 350:
                    gray_orig[labels == i] = 0

            # num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray_orig, connectivity=8)

            # If the center of mass is on black background remove the cluster. 
            # The rebar is shaped in a way that the center of mass is in the middle of the rectangle which is background.
            # for i in range(1, num_labels):
            #     x, y = int(centroids[i][0]), int(centroids[i][1])
            #     if gray_orig[y, x] == 255:
            #         gray_orig[labels == i] = 0


            vertical_angle, horizontal_angle = find_rotation(gray_orig, 0)

            rotated_image = rotate_image("image", gray_orig, horizontal_angle, None, 1, 0)
            rotated_image = cv2.threshold(rotated_image, 0, 255, cv2.THRESH_BINARY)[1]
            #skeleton1 = skeletonize(rotated_image, method='lee').astype(np.uint8)
            rotated_image = cv2.ximgproc.thinning(rotated_image, thinningType=cv2.ximgproc.THINNING_GUOHALL)

            vertical, horizontal = split_horizontal_and_vertical(rotated_image, 5, 0)

            reconstructed_vertical_skeleton = reconstruct_skeleton("vertical", vertical, rotated_image, 3, 7, 0)
            reconstructed_horizontal_skeleton = reconstruct_skeleton("horizontal", horizontal, rotated_image, 3, 7, 0)

            pruned_vertical = remove_small_blobs("vertical", reconstructed_vertical_skeleton, 30, 0)
            pruned_horizontal = remove_small_blobs("horizontal", reconstructed_horizontal_skeleton, 30, 0)
            
            vertical = rotate_image("vertical", pruned_vertical, -horizontal_angle, None, 1, 0)
            vertical = cv2.threshold(vertical, 0, 255, cv2.THRESH_BINARY)[1]
            # vertical = skeletonize(vertical, method='lee').astype(np.uint8)
            vertical = cv2.ximgproc.thinning(vertical, thinningType=cv2.ximgproc.THINNING_GUOHALL)

            horizontal = rotate_image("horizontal", pruned_horizontal, -horizontal_angle, None, 1, 0)
            horizontal = cv2.threshold(horizontal, 0, 255, cv2.THRESH_BINARY)[1]
            # horizontal = skeletonize(horizontal, method='lee').astype(np.uint8)
            horizontal = cv2.ximgproc.thinning(horizontal, thinningType=cv2.ximgproc.THINNING_GUOHALL)


            vertical_num_labels, vertical_labels, vertical_stats, vertical_centroids, vertical_cluster_img = cluster_skeleton("vertical", vertical, 1)
            horizontal_num_labels, horizontal_labels, horizontal_stats, horizontal_centroids, horizontal_cluster_img = cluster_skeleton("horizontal", horizontal, 1)

            closest_pixels_vertical, bboxs_vertical = find_area_of_interest("vertical", vertical_labels, vertical_num_labels, gray_orig, image, 0)

            closest_pixels_horizontal, bboxs_horizontal = find_area_of_interest("horizontal", horizontal_labels, horizontal_num_labels, gray_orig, image, 0)

            # draw a line between the closest pixels
            img = image
            
            print("Number of damages found in vertical direction", len(closest_pixels_vertical))

            print("Number of damages found in horizontal direction", len(closest_pixels_horizontal))

            for bbox in bboxs_vertical:
                img = cv2.rectangle(img, bbox[0], bbox[1], (0, 255, 0), 2)
            for bbox in bboxs_horizontal:
                img = cv2.rectangle(img, bbox[0], bbox[1], (0, 255, 255), 2)

            for closest_pixel in closest_pixels_vertical:
                img = cv2.line(img, closest_pixel[0], closest_pixel[1], (0, 0, 255), 2)
            for closest_pixel in closest_pixels_horizontal:
                img = cv2.line(img, closest_pixel[0], closest_pixel[1], (255, 0, 0), 2)

            # if len(bboxs_vertical) > 0:
            #     img = cv2.line(image, closest_pixels_vertical[0], closest_pixels_vertical[1], (0, 0, 255), 2)    
            #     img = cv2.rectangle(img, bbox_vertical[0], bbox_vertical[1], (0, 255, 0), 2)
            
            # if bbox_horizontal is not None:   
            #     img = cv2.line(image, closest_pixels_horizontal[0], closest_pixels_horizontal[1], (255, 0, 0), 2)
            #     img = cv2.rectangle(img, bbox_horizontal[0], bbox_horizontal[1], (0, 255, 255), 2)

            cv2.imshow(f'final', img)
            cv2.waitKey(1)


            # From the corresponding depth image use the depth value to calculate the camera coordinates
            # of the rebar pixels
            
            K = np.array([[92.76239013671875, 0, 321.00030517578125],    # fx, 0, cx
                    [0, 392.76239013671875, 246.15286254882812],    # 0, fy, cy
                    [0, 0, 1]])                                     # 0, 0, 1

            K = np.array([[465.33203125, 0, 353.9921875],    # fx, 0, cx
                    [0, 465.33203125, 251.28125],    # 0, fy, cy
                    [0, 0, 1]])                                     # 0, 0, 1

            # Inverse of the camera matrix
            K_inv = np.linalg.inv(K)

            print("Closest pixel", closest_pixel)

            # Z = self.depth_image[closest_pixel[0][0], closest_pixel[0][1]] * 0.001
            # print("Depth value of the closest pixel", Z)

            # # Z2 = self.depth_image[closest_pixel[1][0], closest_pixel[1][1]] * 0.001
            # # print("Depth value of the closest pixel", Z2)

            # cords1 = pixel_to_camera(closest_pixel[0][1], closest_pixel[0][0], Z, K_inv)

            # cords2 = pixel_to_camera(closest_pixel[1][1], closest_pixel[1][0], Z, K_inv)

            # calculated_distance = np.sqrt((cords1[0]-cords2[0])**2 + (cords1[1]-cords2[1])**2 + (cords1[2]-cords2[2])**2)

            # print("Calculated distance between the closest pixels", calculated_distance)

            white_pixels = np.column_stack(np.where(img == 255))

            # get a list of pixelcoordinates from each cluster
            vertical_pixel_coordinates = []
            horizontal_pixel_coordinates = []
            for i in range(1, np.max(vertical_labels)+1):
                vertical_pixel_coordinates.append(np.column_stack(np.where(vertical_labels == i)))
            for i in range(1, np.max(horizontal_labels)+1):
                horizontal_pixel_coordinates.append(np.column_stack(np.where(horizontal_labels == i)))

            camera_cluster_coordinates = []

            for cluster in vertical_pixel_coordinates:
                camera_coordinates = []
                for (v, u) in cluster: 
                    Z = self.depth_image[v, u] * 0.001
                    # if Z == 0:
                    #     # Take the average of the surrounding pixels
                    #     Z = np.max([self.depth_image[v-1, u], self.depth_image[v+1, u], self.depth_image[v, u-1], self.depth_image[v, u+1]]) * 0.001
        
                    camera_coords = pixel_to_camera(u, v, Z, K_inv)
                    camera_coordinates.append(camera_coords)
                camera_cluster_coordinates.append(camera_coordinates)

            for cluster in horizontal_pixel_coordinates:
                camera_coordinates = []
                for (v, u) in cluster: 
                    Z = self.depth_image[v, u] * 0.001
                    if Z == 0:
                        # Move one pixel over to the right
                        Z = np.max([self.depth_image[v-1, u], self.depth_image[v+1, u], self.depth_image[v, u-1], self.depth_image[v, u+1]]) * 0.001
                    camera_coords = pixel_to_camera(u, v, Z, K_inv)
                    camera_coordinates.append(camera_coords)
                camera_cluster_coordinates.append(camera_coordinates)

            plot_camera_coordinates(camera_cluster_coordinates, 0)
            
            image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.count += 1
        
       
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    img_sub = ImageSubscriber()
    img_sub.run()



#     file = open("/home/dtu/Desktop/Rebar-segmentation-Ransac/filtered_camera_coordinates.csv",'w')


#         # Fit a line to the points
#         point_on_line, direction_vector = fit_line_to_points(filtered_camera_coordinates)
        
#         print("Point on line:", point_on_line)
#         print("Direction vector:", direction_vector)
        
#         # Create points along the fitted line for plotting
#         line_points = np.array([point_on_line + t * direction_vector for t in np.linspace(-1, 1, 100)])
        
#         # Plot the fitted line
#         ax.plot(line_points[:, 0], line_points[:, 1], line_points[:, 2], 'b')
    

#         # Save the filtered camera coordinates to a csv file. Without the scientific notation and 3 decimal places
#         # insert header with the column names
#         file.write(f'Rebar {i} \n')
#         file.write("point on line,")
#         np.savetxt(file, point_on_line, fmt='%.4f', delimiter=',', header='X,Y,Z', comments='')
#         file.write('\n')
#         file.write("direction vector,")
#         np.savetxt(file, direction_vector, fmt='%.4f', delimiter=',', header='X,Y,Z', comments='')
#         file.write('\n')
#         np.savetxt(file, filtered_camera_coordinates, fmt='%.4f', delimiter=',', header=f'X,Y,Z', comments='')
#         file.write('\n')


#     file.close()