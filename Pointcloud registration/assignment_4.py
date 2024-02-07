import numpy as np
import time
import os

from scipy.spatial import KDTree
from scipy.ndimage import binary_erosion
#from sklearn.neighbors import NearestNeighbors
import random

try:
    import open3d as o3d
    visualize = True
except ImportError:
    print('To visualize you need to install Open3D. \n \t>> You can use "$ pip install open3d"')
    visualize = False

from assignment_4_helper import ICPVisualizer, load_point_cloud, view_point_cloud, quaternion_matrix, \
    quaternion_from_axis_angle, load_pcs_and_camera_poses, save_point_cloud


def transform_point_cloud(point_cloud, t, R):
    """
    Transform a point cloud applying a rotation and a translation
    :param point_cloud: np.arrays of size (N, 6)
    :param t: np.array of size (3,) representing a translation.
    :param R: np.array of size (3,3) representing a 3D rotation matrix.
    :return: np.array of size (N,6) resulting in applying the transformation (t,R) on the point cloud point_cloud.
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    XYZ = point_cloud[:, :3]

    rotated_XYZ = np.dot(XYZ, R.T)
    translated_points = rotated_XYZ + t
    transformed_point_cloud = np.hstack((translated_points, point_cloud[:, 3:]))
    # ------------------------------------------------
    return transformed_point_cloud


def merge_point_clouds(point_clouds, camera_poses):
    """
    Register multiple point clouds into a common reference and merge them into a unique point cloud.
    :param point_clouds: List of np.arrays of size (N_i, 6)
    :param camera_poses: List of tuples (t_i, R_i) representing the camera i pose.
              - t: np.array of size (3,) representing a translation.
              - R: np.array of size (3,3) representing a 3D rotation matrix.
    :return: np.array of size (N, 6) where $$N = sum_{i=1}^K N_i$$
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    # TODO: Vectyorize later: This is SHIT
    transformed_point_clouds = []

    # Iterate through each point cloud and its corresponding pose
    for i, cloud in enumerate(point_clouds):
        t, R = camera_poses[i]
        transformed_point_cloud = transform_point_cloud(cloud, t, R)
        transformed_point_clouds.append(transformed_point_cloud) # np.array of size (N,6)

    # Merge all transformed point clouds
    merged_point_cloud = np.vstack(transformed_point_clouds)
    # ------------------------------------------------
    return merged_point_cloud

def find_closest_points(point_cloud_A, point_cloud_B):
    """
    Find the closest point in point_cloud_B for each element in point_cloud_A.
    :param point_cloud_A: np.array of size (n_a, 6)
    :param point_cloud_B: np.array of size (n_b, 6)
    :return: np.array of size(n_a,) containing the closest point indexes in point_cloud_B
            for each point in point_cloud_A
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    # def Distance(p, q):
    #     dx = p[0] - q[0]
    #     dy = p[1] - q[1]
    #     dz = p[2] - q[2]
    #     return np.sqrt(dx*dx + dy*dy + dz*dz)

    # closest_points_indices = []

    # for point_A in point_cloud_A:
    #     distances = [Distance(point_A[:3], point_B[:3]) for point_B in point_cloud_B]
    #     closest_point_index = np.argmin(distances)
    #     closest_points_indices.append(closest_point_index)

    # neigh = NearestNeighbors(n_neighbors=1)
    # neigh.fit(point_cloud_B)
    # _, indices = neigh.kneighbors(point_cloud_A, return_distance=True)
    #Extract the XYZ coordinates from the point clouds
    xyz_A = point_cloud_A[:, :3]
    xyz_B = point_cloud_B[:, :3]

    # Build KDTree for point_cloud_B
    kdtree_B = KDTree(xyz_B)

    # Query KDTree to find the closest points in B for each point in A
    _, closest_points_indices = kdtree_B.query(xyz_A)

    #return np.array(indices.ravel())
    return closest_points_indices
    # ------------------------------------------------
    #return closest_points_indxs


def find_best_transform(point_cloud_A, point_cloud_B):
    """
    Find the transformation 2 corresponded point clouds.
    Note 1: We assume that each point in the point_cloud_A is corresponded to the point in point_cloud_B at the same location.
        i.e. point_cloud_A[i] is corresponded to point_cloud_B[i] forall 0<=i<N
    :param point_cloud_A: np.array of size (N, 6) (scene)
    :param point_cloud_B: np.array of size (N, 6) (model)
    :return:
         - t: np.array of size (3,) representing a translation between point_cloud_A and point_cloud_B
         - R: np.array of size (3,3) representing a 3D rotation between point_cloud_A and point_cloud_B
    Note 2: We transform the model to match the scene.
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    t = None    # TODO: Replace None with your result
    R = None    # TODO: Replace None with your result
    # 1.
    pmean = np.mean(point_cloud_A[:, :3], axis=0)
    qmean = np.mean(point_cloud_B[:, :3], axis=0)

    X = point_cloud_A[:, :3] - pmean
    Y = point_cloud_B[:, :3] - qmean
    # 2. W matrix
    W = np.dot(X.T, Y)
    # 3. Decompose using svd
    U, _, VT = np.linalg.svd(W)
    
    # 4.
    R = U @ VT
    # 5.
    t = pmean - np.dot(R, qmean)

    # ------------------------------------------------
    return t, R


def icp_step(point_cloud_A, point_cloud_B, t_init, R_init):
    """
    Perform an ICP iteration to find a new estimate of the pose of the model point cloud with respect to the scene pointcloud.
    :param point_cloud_A: np.array of size (N_a, 6) (scene)
    :param point_cloud_B: np.array of size (N_b, 6) (model)
    :param t_init: np.array of size (3,) representing the initial transformation candidate
                    * It may be the output from the previous iteration
    :param R_init: np.array of size (3,3) representing the initial rotation candidate
                    * It may be the output from the previous iteration
    :return:
        - t: np.array of size (3,) representing a translation estimate between point_cloud_A and point_cloud_B
        - R: np.array of size (3,3) representing a 3D rotation estimate between point_cloud_A and point_cloud_B
        - correspondences: np.array of size(n_a,) containing the closest point indexes in point_cloud_B
            for each point in point_cloud_A
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    point_cloud_B_transformed = transform_point_cloud(point_cloud_B, t_init, R_init)
    correspondences = find_closest_points(point_cloud_A, point_cloud_B_transformed)
    
    correspondences_mask = np.array(correspondences)
    t, R = find_best_transform(point_cloud_A, point_cloud_B[correspondences_mask])
    # ------------------------------------------------
    return t, R, correspondences


def icp(point_cloud_A, point_cloud_B, num_iterations=150, t_init=None, R_init=None, visualize=True):
    """
    Find the
    :param point_cloud_A: np.array of size (N_a, 6) (scene)
    :param point_cloud_B: np.array of size (N_b, 6) (model)
    :param num_iterations: <int> number of icp iteration to be performed
    :param t_init: np.array of size (3,) representing the initial transformation candidate
    :param R_init: np.array of size (3,3) representing the initial rotation candidate
    :param visualize: <bool> Whether to visualize the result
    :return:
         - t: np.array of size (3,) representing a translation estimate between point_cloud_A and point_cloud_B
         - R: np.array of size (3,3) representing a 3D rotation estimate between point_cloud_A and point_cloud_B
    """
    if t_init is None:
        t_init = np.zeros(3)
    if R_init is None:
        R_init = np.eye(3)
    if visualize:
        vis = ICPVisualizer(point_cloud_A, point_cloud_B)
    t = t_init
    R = R_init
    correspondences = None  # Initialization waiting for a value to be assigned
    if visualize:
        vis.view_icp(R=R, t=t)
    for i in range(num_iterations):
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        #print(f"{i}")
        t, R, correspondences = icp_step(point_cloud_A, point_cloud_B, t, R)
        # ------------------------------------------------
        if visualize:
            vis.plot_correspondences(correspondences)   # Visualize point correspondences
            time.sleep(.5)  # Wait so we can visualize the correspondences
            vis.view_icp(R, t)  # Visualize icp iteration

    return t, R


# def filter_point_cloud(point_cloud):
#     """
#     Remove unnecessary point given the scene point_cloud.
#     :param point_cloud: np.array of size (N,6)
#     :return: np.array of size (n,6) where n <= N
#     """
#     # ------------------------------------------------
#     # FILL WITH YOUR CODE
#     # WORKSPACE FILTER
#     x_coord = point_cloud[:, 0]
#     y_coord = point_cloud[:, 1]
#     z_coord = point_cloud[:, 2]
#     rgb_values = point_cloud[:, 3:6]

#     valid_x_range = [-0.35, 0.35]
#     valid_y_range = [-0.35, 0.35]
#     valid_z_range = [0.01, np.inf]

#     rgb_values = point_cloud[:, 3:6]

#     color_range_pink = [170., 112., 122., 252., 187., 182. ]  # [min_r, min_g, min_b, max_r, max_g, max_b]
#     #color_range = [241., 155., 149., 101., 47., 45.] # old
#     color_range_meroon = [91., 41., 40.,170., 71., 69. ] 

#     # color_range_yellow1 = np.array([200, 200, 0, 255, 255, 108]) / 256
#     # color_range_yellow2 = np.array([95, 95, 0, 200, 200, 0]) / 256

#     color_range_yellow1 = np.array([160, 100, 35, 240, 214, 185]) / 256
#     color_range_yellow2 = np.array([60, 37, 13, 160, 120, 35]) / 256
    

#     color_mask_yellow_1 = np.logical_and.reduce([
#         rgb_values[:, 0] >= color_range_yellow1[0]/ 256.,  # min_r
#         rgb_values[:, 1] >= color_range_yellow1[1]/ 256.,  # min_g
#         rgb_values[:, 2] >= color_range_yellow1[2]/ 256.,  # min_b
#         rgb_values[:, 0] <= color_range_yellow1[3]/ 256.,  # max_r
#         rgb_values[:, 1] <= color_range_yellow1[4]/ 256.,  # max_g
#         rgb_values[:, 2] <= color_range_yellow1[5]/ 256.,  # max_b
#     ])

#     color_mask_yellow_2 = np.logical_and.reduce([
#         rgb_values[:, 0] >= color_range_yellow2[0]/ 256.,  # min_r
#         rgb_values[:, 1] >= color_range_yellow2[1]/ 256.,  # min_g
#         rgb_values[:, 2] >= color_range_yellow2[2]/ 256.,  # min_b
#         rgb_values[:, 0] <= color_range_yellow2[3]/ 256.,  # max_r
#         rgb_values[:, 1] <= color_range_yellow2[4]/ 256.,  # max_g
#         rgb_values[:, 2] <= color_range_yellow2[5]/ 256.,  # max_b
#     ])


#     color_mask_yellow = np.logical_or(color_mask_yellow_1, color_mask_yellow_2) # Contains yellow

#     color_mask_pink = np.logical_and.reduce([
#         rgb_values[:, 0] >= color_range_pink[0]/ 256.,  # min_r
#         rgb_values[:, 1] >= color_range_pink[1]/ 256.,  # min_g
#         rgb_values[:, 2] >= color_range_pink[2]/ 256.,  # min_b
#         rgb_values[:, 0] <= color_range_pink[3]/ 256.,  # max_r
#         rgb_values[:, 1] <= color_range_pink[4]/ 256.,  # max_g
#         rgb_values[:, 2] <= color_range_pink[5]/ 256.,  # max_b
#     ])

#     color_mask_meroon = np.logical_and.reduce([
#         rgb_values[:, 0] >= color_range_meroon[0]/ 256.,  # min_r
#         rgb_values[:, 1] >= color_range_meroon[1]/ 256.,  # min_g
#         rgb_values[:, 2] >= color_range_meroon[2]/ 256.,  # min_b
#         rgb_values[:, 0] <= color_range_meroon[3]/ 256.,  # max_r
#         rgb_values[:, 1] <= color_range_meroon[4]/ 256.,  # max_g
#         rgb_values[:, 2] <= color_range_meroon[5]/ 256.,  # max_b
#     ])

#     color_mask_red = np.logical_not(np.logical_or(color_mask_pink, color_mask_meroon)) # Doesn't contain red
#     color_mask_yellow = np.logical_not(np.logical_or(color_mask_yellow_1, color_mask_yellow_2))
    
#     color_mask = np.logical_or(color_mask_yellow, color_mask_red)
#     # # mask to filter 
#     x_mask = np.logical_and(x_coord >= valid_x_range[0], x_coord <= valid_x_range[1])
#     y_mask = np.logical_and(y_coord >= valid_y_range[0], y_coord <= valid_y_range[1])
#     z_mask = np.logical_and(z_coord >= valid_z_range[0], z_coord <= valid_z_range[1])

#     mask_dimensionality = color_mask_red.ndim
#     structuring_element = np.ones((7,)*mask_dimensionality, dtype=bool)  # Adjust the size as needed
#     # Perform dilation
#     expanded_mask = binary_erosion(color_mask_red, structure=structuring_element)

#     valid_points_mask = np.logical_and.reduce([x_mask,y_mask,z_mask, expanded_mask])

#     # # Filter points based on the mask
#     filtered_pc = point_cloud[valid_points_mask]

#     # ------------------------------------------------
#     return filtered_pc

def filter_point_cloud(point_cloud):
    x_mask = np.logical_and(point_cloud[:,0] >= -0.45, point_cloud[:,0] <= 0.45)
    y_mask = np.logical_and(point_cloud[:,1] >= -0.45, point_cloud[:,1] <= 0.45)
    valid_z_range = [0.01, np.inf]
    z_mask = np.logical_and(point_cloud[:,2] >= valid_z_range[0], point_cloud[:,2] <= valid_z_range[1])
    logical_mask = np.logical_and.reduce([x_mask, y_mask,z_mask])
    def rgb_to_hsv(arr):
        arr = np.asarray(arr)
        
        in_shape = arr.shape
        arr = np.array(
            arr, copy=False,
            dtype=np.promote_types(arr.dtype, np.float32),  # Don't work on ints.
            ndmin=2,  # In case input was 1D.
        )
        out = np.zeros_like(arr)
        arr_max = arr.max(-1)
        ipos = arr_max > 0
        delta = np.ptp(arr, -1)
        s = np.zeros_like(delta)
        s[ipos] = delta[ipos] / arr_max[ipos]
        ipos = delta > 0
        # red is max
        idx = (arr[..., 0] == arr_max) & ipos
        out[idx, 0] = (arr[idx, 1] - arr[idx, 2]) / delta[idx]
        # green is max
        idx = (arr[..., 1] == arr_max) & ipos
        out[idx, 0] = 2. + (arr[idx, 2] - arr[idx, 0]) / delta[idx]
        # blue is max
        idx = (arr[..., 2] == arr_max) & ipos
        out[idx, 0] = 4. + (arr[idx, 0] - arr[idx, 1]) / delta[idx]
        out[..., 0] = (out[..., 0] / 6.0) % 1.0
        out[..., 1] = s
        out[..., 2] = arr_max
        return out.reshape(in_shape)
    hsv = rgb_to_hsv(point_cloud[:,3:])
    color_mask = np.logical_and(hsv[:,0]>0.125, hsv[:,0]<0.25)
    mask = np.logical_and(logical_mask, color_mask)
    filtered_pc = point_cloud[mask]
    return filtered_pc

def ransac_icp_step(point_cloud_A, point_cloud_B, t_init, R_init, ransac_iters = 10, ransac_thresh=0.1):
    """
    Perform an ICP iteration to find a new estimate of the pose of the model point cloud with respect to the scene pointcloud.
    :param point_cloud_A: np.array of size (N_a, 6) (scene)
    :param point_cloud_B: np.array of size (N_b, 6) (model)
    :param t_init: np.array of size (3,) representing the initial transformation candidate
                    * It may be the output from the previous iteration
    :param R_init: np.array of size (3,3) representing the initial rotation candidate
                    * It may be the output from the previous iteration
    
    :return:
        - t: np.array of size (3,) representing a translation estimate between point_cloud_A and point_cloud_B
        - R: np.array of size (3,3) representing a 3D rotation estimate between point_cloud_A and point_cloud_B
        - correspondences: np.array of size(n_a,) containing the closest point indexes in point_cloud_B
            for each point in point_cloud_A
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    best_inliers = []
    best_t = np.zeros(3)
    best_R = np.eye(3)

    point_cloud_B_transformed = transform_point_cloud(point_cloud_B, t_init, R_init)
    # correspondences = find_closest_points(point_cloud_A, point_cloud_B_transformed)
    
    # correspondences_mask = np.array(correspondences)
    # t, R = find_best_transform(point_cloud_A, point_cloud_B[correspondences_mask])

    #############
    for ransac_iter in range(ransac_iters):
        # Randomly sample a subset of correspondences
        sampled_indices = random.sample(range(len(point_cloud_A)), k=min(3, len(point_cloud_A)))
        sampled_correspondences = np.array(sampled_indices)

        # Find correspondence with sample 
        kdtree_B = KDTree(point_cloud_B_transformed[sampled_correspondences][:, :3])

        # Query KDTree to find the closest points in B for each point in A
        distances, correspondences = kdtree_B.query(point_cloud_A[sampled_correspondences][:,:3])

         # Estimate the transformation using the sampled correspondences
        correspondences_mask = np.array(correspondences)
        t_ransac, R_ransac = find_best_transform(point_cloud_A[correspondences_mask], point_cloud_B[correspondences_mask])

        # Use RANSAC to find inliers
        inliers = distances.flatten() < ransac_thresh
        inliers_indices = np.nonzero(inliers)[0]

        # Check if the current set of inliers is the best
        if len(inliers_indices) > len(best_inliers):
            best_inliers = inliers_indices
            best_t = t_ransac
            best_R = R_ransac
    
    # Final transformation estimation using all inliers
    t, R = find_best_transform(point_cloud_A[best_inliers], point_cloud_B[best_inliers])

    correspondences = best_inliers
    # ------------------------------------------------
    return t, R, correspondences

def custom_icp(point_cloud_A, point_cloud_B, num_iterations=50, t_init=None, R_init=None, visualize=True):
    """
        Find the
        :param point_cloud_A: np.array of size (N_a, 6) (scene)
        :param point_cloud_B: np.array of size (N_b, 6) (model)
        :param num_iterations: <int> number of icp iteration to be performed
        :param t_init: np.array of size (3,) representing the initial transformation candidate
        :param R_init: np.array of size (3,3) representing the initial rotation candidate
        :param visualize: <bool> Whether to visualize the result
        :return:
             - t: np.array of size (3,) representing a translation estimate between point_cloud_A and point_cloud_B
             - R: np.array of size (3,3) representing a 3D rotation estimate between point_cloud_A and point_cloud_B
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE (OPTIONAL)
    # Perform RANSAC

    #t, R = icp(point_cloud_A, point_cloud_B, num_iterations=num_iterations, t_init=t_init, R_init=R_init, visualize=visualize)  #TODO: Edit as needed (optional)
    if t_init is None:
        t_init = np.zeros(3)
    if R_init is None:
        R_init = np.eye(3)
    if visualize:
        vis = ICPVisualizer(point_cloud_A, point_cloud_B)
    t = t_init
    R = R_init
    correspondences = None  # Initialization waiting for a value to be assigned
    if visualize:
        vis.view_icp(R=R, t=t)
    for i in range(num_iterations):
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        # print(f"{i}")
        # t, R, correspondences = ransac_icp_step(point_cloud_A, point_cloud_B, t, R)

        # t, R = icp(point_cloud_A[correspondences], point_cloud_B[correspondences])
        t, R, correspondences = icp_step(point_cloud_A, point_cloud_B, t, R)
        # ------------------------------------------------
        if visualize:
            vis.plot_correspondences(correspondences)   # Visualize point correspondences
            time.sleep(.5)  # Wait so we can visualize the correspondences
            vis.view_icp(R, t)  # Visualize icp iteration
    # ------------------------------------------------
    return t, R
    #pass



# ===========================================================================

# Test functions:

def transform_point_cloud_example(path_to_pointcloud_files, visualize=True):
    pc_source = load_point_cloud(os.path.join(path_to_pointcloud_files, 'michigan_M_med.ply'))  # Source
    pc_goal = load_point_cloud(os.path.join(path_to_pointcloud_files, 'michigan_M_med_tr.ply'))  # Transformed Goal
    t_gth = np.array([-0.5, 0.5, -0.2])
    r_angle = np.pi / 3
    R_gth = quaternion_matrix(np.array([np.cos(r_angle / 2), 0, np.sin(r_angle / 2), 0]))
    pc_tr = transform_point_cloud(pc_source, t=t_gth, R=R_gth)  # Apply your transformation to the source point cloud
    # Paint the transformed in red
    pc_tr[:, 3:] = np.array([.73, .21, .1]) * np.ones((pc_tr.shape[0], 3))  # Paint it red
    if visualize:
        # Visualize first without transformation
        print('Printing the source and goal point clouds')
        view_point_cloud([pc_source, pc_goal])
        # Visualize the transformation
        print('Printing the transformed output (in red) along source and goal point clouds')
        view_point_cloud([pc_source, pc_goal, pc_tr])
    else:
        # Save the pc so we can visualize them using other software
        save_point_cloud(np.concatenate([pc_source, pc_goal], axis=0), 'tr_pc_example_no_transformation',
                     path_to_pointcloud_files)
        save_point_cloud(np.concatenate([pc_source, pc_goal, pc_tr], axis=0), 'tr_pc_example_transform_applied',
                     path_to_pointcloud_files)
        print('Transformed point clouds saved as we cannot visualize them.\n Use software such as Meshlab to visualize them.')


def reconstruct_scene(path_to_pointcloud_files, visualize=True):
    pcs, camera_poses = load_pcs_and_camera_poses(path_to_pointcloud_files)
    pc_reconstructed = merge_point_clouds(pcs, camera_poses)
    if visualize:
        print('Displaying reconstructed point cloud scene.')
        view_point_cloud(pc_reconstructed)
    else:
        print(
            'Reconstructed scene point clouds saved as we cannot visualize it.\n Use software such as Meshlab to visualize them.')
        save_point_cloud(pc_reconstructed, 'reconstructed_scene_pc', path_to_pointcloud_files)


def perfect_model_icp(path_to_pointcloud_files, visualize=True):
    # Load the model
    pcB = load_point_cloud(os.path.join(path_to_pointcloud_files, 'michigan_M_med.ply'))  # Model
    pcB[:, 3:] = np.array([.73, .21, .1]) * np.ones((pcB.shape[0], 3))  # Paint it red
    pcA = load_point_cloud(os.path.join(path_to_pointcloud_files, 'michigan_M_med.ply'))  # Perfect scene
    # Apply transfomation to scene so they differ
    t_gth = np.array([0.4, -0.2, 0.2])
    r_angle = np.pi / 2
    R_gth = quaternion_matrix(np.array([np.cos(r_angle / 2), 0, np.sin(r_angle / 2), 0]))
    pcA = transform_point_cloud(pcA, R=R_gth, t=t_gth)
    R_init = np.eye(3)
    t_init = np.mean(pcA[:, :3], axis=0)

    # ICP -----
    t, R = icp(pcA, pcB, num_iterations=70, t_init=t_init, R_init=R_init, visualize=visualize)
    print('Infered Position: ', t)
    print('Infered Orientation:', R)
    print('\tReal Position: ', t_gth)
    print('\tReal Orientation:', R_gth)


def real_model_icp(path_to_pointcloud_files, visualize=True):
    # Load the model
    pcB = load_point_cloud(os.path.join(path_to_pointcloud_files, 'michigan_M_med.ply'))  # Model
    pcB[:, 3:] = np.array([.73, .21, .1]) * np.ones((pcB.shape[0], 3)) # Paint it red
    # ------ Noisy partial view scene -----
    pcs, camera_poses = load_pcs_and_camera_poses(path_to_pointcloud_files)
    pc = merge_point_clouds(pcs, camera_poses)
    pcA = filter_point_cloud(pc)
    #pcB = filter_point_cloud(pcB)
    print(np.shape(pcA))
    #print(np.shape(pcB))
    if visualize:
        print('Displaying filtered point cloud. Close the window to continue.')
        view_point_cloud(pcA)
    else:
        print('Filtered scene point clouds saved as we cannot visualize it.\n Use software such as Meshlab to visualize them.')
        save_point_cloud(pcA, 'filtered_scene_pc', path_to_pointcloud_files)
    R_init = quaternion_matrix(quaternion_from_axis_angle(axis=np.array([0, 0, 1]), angle=np.pi / 2))
    t_init = np.mean(pcA[:, :3], axis=0)
    t_init[-1] = 0
    t, R = custom_icp(pcA, pcB, num_iterations=70, t_init=t_init, R_init=R_init)
    print('Infered Position: ', t)
    print('Infered Orientation:', R)


if __name__ == '__main__':
    # by default we assume that the point cloud files are on the same directory

    path_to_files = 'a4_pointcloud_files' # TODO: Change the path to the directory containing your point cloud files

    # Test for part 1
    # transform_point_cloud_example(path_to_files, visualize=visualize) # TODO: Uncomment to test

    # Test for part 2
    # reconstruct_scene(path_to_files, visualize=visualize) # TODO: Uncomment to test
    #visualize = False
    # Test for part 5
    # perfect_model_icp(path_to_files, visualize=visualize) # TODO: Uncomment to test

    # Test for part 6
    real_model_icp(path_to_files, visualize=visualize)    # TODO: Uncomment to test

