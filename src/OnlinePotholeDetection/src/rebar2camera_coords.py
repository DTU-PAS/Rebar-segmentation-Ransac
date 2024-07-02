# 
import rospy
import cv2
import numpy as np
import random
from skimage.morphology import skeletonize
from scipy.linalg import svd
import matplotlib.pyplot as plt
from plantcv import plantcv as pcv
from sensor_msgs.msg import Image
from std_msgs.msg import String
import struct
from cv_bridge import CvBridge, CvBridgeError
import concurrent.futures
import time

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

def reconstruct_skeleton(name, src, orig,  ksize = 3, debug_level = 0):
    this_iteration = src
    last_iteration = src
    for i in range(15):
        this_iteration = cv2.dilate(last_iteration, np.ones((ksize,ksize), np.uint8), iterations = 1)
        # Show the image at this iteration
        if debug_level == 2:
            cv2.imshow(f'reconstruction {name} - dialted', this_iteration)
            cv2.waitKey(1)
        this_iteration = this_iteration & orig
        # Show the image at this iteration
        if debug_level == 2:
            cv2.imshow(f'reconstruction {name} - masked', this_iteration)
            cv2.waitKey(1)
        if np.array_equal(last_iteration, this_iteration):
            # convergence!
            break
        last_iteration = this_iteration.copy()
    
    
    skeleton, segmented_img, segment_objects = pcv.morphology.prune(skel_img=this_iteration, size=100, mask=orig)
    if debug_level == 1 or debug_level == 2:
        cv2.imshow(f'reconstruction {name} - pruned', skeleton)
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
    return cluster_img, labels

def pixel_to_camera(u, v, Z, K_inv):
    # Normalize pixel coordinates
    pixel_coords = np.array([u, v, 1])
    camera_coords = Z * K_inv.dot(pixel_coords)
    return camera_coords

def remove_small_blobs(name, img, min_blob_size, debug_level=0):
    # Find connected components
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)

    # Create an output image to store the result
    output = np.zeros_like(img)

    # Loop through all the detected components
    for i in range(1, num_labels):  # Start from 1 to skip the background
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_blob_size:
            # Retain the component if its area is larger than the minimum blob size
            component_mask = (labels == i).astype(np.uint8) * 255
            output[component_mask == 255] = img[component_mask == 255]
    if debug_level:
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
        cv2.imshow('pruned_vertical', pruned_vertical)
        cv2.waitKey(1)
        cv2.imshow('pruned_horizontal', pruned_horizontal)
        cv2.waitKey(1)
    return pruned_vertical, pruned_horizontal


class ImageSubscriber:
    def __init__(self):
        self.br = CvBridge()
        # Subscribe to the depth and color image topics
        #self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.color_sub = rospy.Subscriber('/label', Image, self.color_callback)
        
        self.depth_image = None
        self.color_image = None

    def depth_callback(self, data):
        try:
            # Convert depth image to numpy array
            dtype = np.dtype('uint16')  # Change this according to the depth image type
            depth_image = np.frombuffer(data.data, dtype=dtype).reshape(data.height, data.width)
            # calcualted_depth = depth_image[150, 98] * 0.001
            # rospy.loginfo(f"Received depth image: {calcualted_depth}")
            self.depth_image = depth_image
            # rospy.loginfo("Received depth image")
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def color_callback(self, data):
        #try:
        self.image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.findRebars()
        # rospy.loginfo("Received color image")
        # except Exception as e:
        #     rospy.logerr(f"Error converting color image: {e}")

    def findRebars(self):
        gray_orig = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        gray_orig = cv2.dilate(gray_orig, np.ones((3,3), np.uint8), iterations=1)
        gray_orig = cv2.erode(gray_orig, np.ones((3,3), np.uint8), iterations=1)

        small_obj_removed = open_by_reconstruction(gray_orig, 4, 4, False)

        skeleton = skeletonize(small_obj_removed, method='lee').astype(np.uint8)

        pruned_vertical, pruned_horizontal = split_horizontal_and_vertical(skeleton, 0)

        pruned_vertical2 = remove_small_blobs("vertical", pruned_vertical, 20, 0)
        pruned_horizontal2 = remove_small_blobs("horizontal", pruned_horizontal, 20, 0)

        reconstructed_vertical_skeleton = reconstruct_skeleton("vertical", pruned_vertical2, skeleton, 3, 0)
        reconstructed_horizontal_skeleton = reconstruct_skeleton("horizontal", pruned_horizontal2, skeleton, 3, 0)

        vertical_clusters, vertical_labels = cluster_skeleton("vertical", reconstructed_vertical_skeleton, 0)
        horizontal_clusters, horizontal_labels = cluster_skeleton("horizontal", reconstructed_horizontal_skeleton, 0)

        # translate every white pixel to a point in camera frame
        # Camera intrinsic parameters
        K = np.array([[92.76239013671875, 0, 321.00030517578125],    # fx, 0, cx
                    [0, 392.76239013671875, 246.15286254882812],    # 0, fy, cy
                    [0, 0, 1]])                                     # 0, 0, 1

        # Inverse of the camera matrix
        K_inv = np.linalg.inv(K)

        # TODO This needs to be the real distance from the camera to the rebars
        # Either use the pcd or the depth image to get the distance
        Z = 0.34

        # white_pixels = np.column_stack(np.where(skeleton == 255))

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
                camera_coords = pixel_to_camera(u, v, Z, K_inv)
                camera_coordinates.append(camera_coords)
            camera_cluster_coordinates.append(camera_coordinates)

        for cluster in horizontal_pixel_coordinates:
            camera_coordinates = []
            for (v, u) in cluster: 
                camera_coords = pixel_to_camera(u, v, Z, K_inv)
                camera_coordinates.append(camera_coords)
            camera_cluster_coordinates.append(camera_coordinates)


        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Set labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        ax.set_xlim([-1.2, 1.2])
        ax.set_ylim([-0.3, 0.3])
        ax.set_zlim([0, 0.7])
        

        for coord in camera_cluster_coordinates:
            coord = np.array(coord)
            if coord.size == 0:
                print(f"No white pixels found for rebar {i}")
                continue

            #line_fit = Line.best_fit(points)
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

            # Fit a line to the points
            # point_on_line, direction_vector = fit_line_to_points(filtered_camera_coordinates)
            point_on_line, direction_vector = fit_line_to_points(coord)
            
            print("Point on line:", point_on_line)
            print("Direction vector:", direction_vector)
            
            # Create points along the fitted line for plotting
            line_points = np.array([point_on_line + t * direction_vector for t in np.linspace(-1, 1, 100)])
            
            # Plot the fitted line
            ax.plot(line_points[:, 0], line_points[:, 1], line_points[:, 2], 'b')
        

            # Save the filtered camera coordinates to a csv file. Without the scientific notation and 3 decimal places
            # insert header with the column names
            # file.write(f'Rebar {i} \n')
            # file.write("point on line,")
            # np.savetxt(file, point_on_line, fmt='%.4f', delimiter=',', header='X,Y,Z', comments='')
            # file.write('\n')
            # file.write("direction vector,")
            # np.savetxt(file, direction_vector, fmt='%.4f', delimiter=',', header='X,Y,Z', comments='')
            # file.write('\n')
            # np.savetxt(file, filtered_camera_coordinates, fmt='%.4f', delimiter=',', header=f'X,Y,Z', comments='')
            # file.write('\n')

            # ax.scatter(filtered_X, filtered_Y, filtered_Z, c=np.random.rand(3,), marker='o', label='%s Rebar' % i)
            ax.scatter(X, Y, Z, c=np.random.rand(3,), marker='o', label='%s Rebar' % i)
            i += 1


            # Show plot
        plt.legend()
        plt.show()

        # file.close()
        
       
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    img_sub = ImageSubscriber()
    img_sub.run()


# # Example usage
# if __name__ == "__main__":
#     # open the image binary image
#     img_orig = cv2.imread('/home/dtu/Desktop/Rebar-segmentation-Ransac/data/Pictures/ransac_1.png', cv2.IMREAD_COLOR)
#     #rs.get_depth_frame()

#     # cv2.imshow('original', img_orig)
#     # cv2.waitKey(1)
    
#     gray_orig = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
#     # cv2.imshow('original', gray_orig)
#     # cv2.waitKey(1)
#     gray_orig = cv2.dilate(gray_orig, np.ones((3,3), np.uint8), iterations=1)
#     gray_orig = cv2.erode(gray_orig, np.ones((3,3), np.uint8), iterations=1)
#     height, width = gray_orig.shape


#     # Erode the picture so much that all small objects are removed
#     # Next reconstruct the image by dialating it again and masking it with the original image
#     small_obj_removed = open_by_reconstruction(gray_orig, 4, 4, False)

#     # Fill in the gaps of the ransac segmentation
#     #dialation = cv2.dilate(small_obj_removed, np.ones((3,3), np.uint8), iterations=1)
#     #erosion = cv2.erode(dialation, np.ones((3,3), np.uint8), iterations=1)

#     # cv2.imshow('Some stuff removed', erosion)
#     # cv2.waitKey(1)

#     skeleton = skeletonize(small_obj_removed, method='lee').astype(np.uint8)
#     skeleton = 255 * skeleton

#     # cv2.imshow('skeleton_orig', skeleton)
#     # cv2.waitKey(1)

#     # skeleton, segmented_img, segment_objects = pcv.morphology.prune(skel_img=skeleton, 
#     #                                                                        size=100, 
#     #                                                                        mask=gray)

#     # cv2.imshow('skeleton_pruned', skeleton)
#     # cv2.waitKey(1)


#     pruned_vertical, pruned_horizontal = skeleton.copy(), skeleton.copy()

#     # This approach works with perfect vertical and horizontal lines 
#     # It will not work with diagonal lines
#     # Use the PCA to find the principal axis of each pixel in the skeleton
#     # This will give us pixels with x, y and direction.
#     # Using the direction we can cluster the pixels and segment the skeleton into branches

#     # Loop through each pixel in the vertical direction
#     for y in range(height):
#         for x in range(width):
#             if skeleton[y, x] == 255:
#                 # check next lines in the vertical direction
#                 for i in range(1, 10):
#                     if y+i < height:
#                         if skeleton[y+i, x] == 255:
#                             pruned_horizontal[y, x] = 0
#                             break
#                     if y-i >= 0:
#                         if skeleton[y-i, x] == 255:
#                             pruned_horizontal[y, x] = 0
#                             break
#                 # check next lines in the horizontal direction
#                 for i in range(1, 10):
#                     if x+i < width:
#                         if skeleton[y, x+i] == 255:
#                             pruned_vertical[y, x] = 0
#                             break
#                     if x-i >= 0:
#                         if skeleton[y, x-i] == 255:
#                             pruned_vertical[y, x] = 0
#                             break

#     # cv2.imshow('pruned_vertical', pruned_vertical)
#     # cv2.waitKey(1)

#     # cv2.imshow('pruned_horizontal', pruned_horizontal)
#     # cv2.waitKey(1)

#     # Remove small objects

#     pruned_vertical2 = remove_small_blobs(pruned_vertical, min_blob_size=20)
#     pruned_horizontal2 = remove_small_blobs(pruned_horizontal, min_blob_size=20)

#     # cv2.imshow('Small blobs removed vertical', pruned_vertical2)
#     # cv2.waitKey(1)

#     # cv2.imshow('small blobs removed horizontal', pruned_horizontal2)
#     # cv2.waitKey(1)

#     reconstructed_vertical_skeleton = reconstruct_skeleton(pruned_vertical2, skeleton, 3, True)


#     reconstructed_horizontal_skeleton = reconstruct_skeleton(pruned_horizontal2, skeleton, 3, True)

#     # cv2.imshow('reconstructed_vertical', reconstructed_vertical_skeleton)
#     # cv2.waitKey(1)

#     # cv2.imshow('reconstructed_horizontal', reconstructed_horizontal_skeleton)
#     # cv2.waitKey(1)

#     # cluster connected pixels
#     # Find connected components
#     vertical_clusters, vertical_labels = cluster_skeleton(reconstructed_vertical_skeleton)
#     cv2.imshow('clustered_vertical', vertical_clusters)
#     cv2.waitKey(1)

#     horizontal_clusters, horizontal_labels = cluster_skeleton(reconstructed_horizontal_skeleton)
#     cv2.imshow('clustered_horizontal', horizontal_clusters)
#     cv2.waitKey(1)



#     # draw found skeleton on roginial image in red
#     skeleton_rgb = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
#     skeleton_rgb[np.where((skeleton_rgb == [255,255,255]).all(axis=2))] = [0,0,255]

#     # merge with original image
#     merged = cv2.addWeighted(img_orig, 0.5, vertical_clusters, 1, 0)
#     merged = cv2.addWeighted(merged, 0.5, horizontal_clusters, 1, 0)

#     # # show the image
#     # cv2.imshow('skeleton on original', merged)
#     # cv2.waitKey(1)


#     # translate every white pixel to a point in camera frame
#     # Camera intrinsic parameters
#     K = np.array([[92.76239013671875, 0, 321.00030517578125],    # fx, 0, cx
#                 [0, 392.76239013671875, 246.15286254882812],    # 0, fy, cy
#                 [0, 0, 1]])        # 0, 0, 1

#     # Inverse of the camera matrix
#     K_inv = np.linalg.inv(K)

#     # TODO This needs to be the real distance from the camera to the rebars
#     # Either use the pcd or the depth image to get the distance
#     Z = 0.34

#     # white_pixels = np.column_stack(np.where(skeleton == 255))

#     # get a list of pixelcoordinates from each cluster
#     vertical_pixel_coordinates = []
#     horizontal_pixel_coordinates = []
#     for i in range(1, np.max(vertical_labels)+1):
#         vertical_pixel_coordinates.append(np.column_stack(np.where(vertical_labels == i)))
#     for i in range(1, np.max(horizontal_labels)+1):
#         horizontal_pixel_coordinates.append(np.column_stack(np.where(horizontal_labels == i)))



#     camera_cluster_coordinates = []

#     for cluster in vertical_pixel_coordinates:
#         camera_coordinates = []
#         for (v, u) in cluster: 
#             camera_coords = pixel_to_camera(u, v, Z, K_inv)
#             camera_coordinates.append(camera_coords)
#         camera_cluster_coordinates.append(camera_coordinates)

#     for cluster in horizontal_pixel_coordinates:
#         camera_coordinates = []
#         for (v, u) in cluster: 
#             camera_coords = pixel_to_camera(u, v, Z, K_inv)
#             camera_coordinates.append(camera_coords)
#         camera_cluster_coordinates.append(camera_coordinates)


#     file = open("/home/dtu/Desktop/Rebar-segmentation-Ransac/filtered_camera_coordinates.csv",'w')


#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
    
#     # Set labels
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
    
#     ax.set_xlim([-1.2, 1.2])
#     ax.set_ylim([-0.3, 0.3])
#     ax.set_zlim([0, 0.7])
    

#     for coord in camera_cluster_coordinates:
#         coord = np.array(coord)
#         #line_fit = Line.best_fit(points)
#         X = coord[:, 0]
#         Y = coord[:, 1]
#         Z = coord[:, 2]

#         # TODO This should not be hardcoded. My first idea is improving the ransac segmentation. 
#         mask = X <= 0.8

#         # # Filter camera coordinates using the mask
#         filtered_camera_coordinates = coord[mask]

#         if filtered_camera_coordinates.size == 0:
#             print(f"No white pixels found for rebar {i}")
#             continue

#         # # If you want to extract the filtered X, Y, Z separately
#         filtered_X = X[mask]
#         filtered_Y = Y[mask]
#         filtered_Z = Z[mask]

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

#         ax.scatter(filtered_X, filtered_Y, filtered_Z, c=np.random.rand(3,), marker='o', label='%s Rebar' % i)
#         i += 1


#         # Show plot
#     plt.legend()
#     plt.show()

#     file.close()

#     cv2.destroyAllWindows()
