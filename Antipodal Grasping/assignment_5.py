import numpy as np
import pybullet as p
import open3d as o3d
import assignment_5_helper as helper

from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt


def get_antipodal(pcd):
    """
    function to compute antipodal grasp given point cloud pcd
    :param pcd: point cloud in open3d format (converted to numpy below)
    :return: gripper pose (4, ) numpy array of gripper pose (x, y, z, theta)
    """
    # convert pcd to numpy arrays of points and normals
    pc_points = np.asarray(pcd.points)
    pc_normals = np.asarray(pcd.normals)

    # ------------------------------------------------
    # FILL WITH YOUR CODE

    # gripper orientation - replace 0. with your calculations
    theta = 0.
    # gripper pose: (x, y, z, theta) - replace 0. with your calculations
    gripper_pose = np.array([0., 0., 0., theta])

     # Gripper width
    gripper_width = 0.15 
    # Concatenate points and normals to create a feature matrix
    features = np.hstack((pc_points, pc_normals))
    # Convert the feature matrix to an Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(features[:, :3])
    pcd.normals = o3d.utility.Vector3dVector(features[:, 3:])

    # Perform DBSCAN clustering
    object_labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=50, print_progress=True))

    # Visualize the clustered point cloud
    # colors = plt.cm.Spectral(object_labels / object_labels.max())
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([pcd])

    # Loop through every object cluster: FIXME :  MAYBE WE JUST TAKE 1st? jsp
    # for i in object_labels:
    i = 0
    object_cluster_mask = object_labels == i
    object_cluster_points = pc_points[object_cluster_mask]
    object_cluster_normals = pc_normals[object_cluster_mask]

    # Cluster points based on the surface normals using DBSCAN

    clustering = DBSCAN(eps=0.1, min_samples=50).fit(object_cluster_normals)
    plane_labels = clustering.labels_

    # Remove outliers (-1 label) from the clustering results
    valid_indices = np.where(plane_labels != -1)[0]
    pc_points = object_cluster_points[valid_indices]
    pc_normals = object_cluster_normals[valid_indices]
    plane_labels = plane_labels[valid_indices]

    print("Number of unique clusters : {}".format(np.unique(plane_labels))) # 8 clusters

    # NOTE: Uncomment to vizualize each plane cluster one by one : jsp
    # plane_cluster = 0
    # # Loop through every plane cluster : We have just the planes 
    # for plane_cluster in range(len(np.unique(plane_labels))):
    #     # Find indices of points belonging to the target cluster
    #     cluster_indices = np.where(plane_labels == plane_cluster)[0]

    #     # Extract points and normals for the target cluster
    #     cluster_points = pc_points[cluster_indices]
    #     cluster_normals = pc_normals[cluster_indices]
    #     print(f"Shape of cluster {plane_cluster}: {np.shape(cluster_points)}")

    #     # VISUALIZATION 
    #     cluster_pcd = o3d.geometry.PointCloud()
    #     cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
    #     cluster_pcd.normals = o3d.utility.Vector3dVector(cluster_normals)

    #     # Visualize the cluster
    #     helper.draw_pc(cluster_pcd)

    # 1.1 Get average of normals
    average_normals = []

    for plane_index in range(len(np.unique(plane_labels))):
        cluster_indices = np.where(plane_labels == plane_index)[0]
        cluster_normals = pc_normals[cluster_indices]
        average_normal = np.mean(cluster_normals, axis=0)
        average_normals.append(average_normal)

    print("Plane surface normals : {}".format(average_normals))

    # 2. Find pairs of anti-parallel planes 
    plane_pairs = []

    for i in range(len(average_normals)):
        for j in range(i+1, len(average_normals)):
            dot_product = np.dot(average_normals[i], average_normals[j])
            
            anti_parallel_threshold = -0.90
            # 2.1 TODO: Compute the orthogonal distance between the two planes
            if dot_product < anti_parallel_threshold:
                plane_pairs.append((i, j))
    
    print("Plane pairs : {}".format(plane_pairs))
    # 2.1. Remove pairs whose minimum distance is greater than gripper width
    smallest_dist = np.inf
    for pair in plane_pairs:
        min_distances = []
        plane_ind_1, plane_ind_2 = pair
        plane_1_points = pc_points[plane_labels == plane_ind_1]
        plane_2_points = pc_points[plane_labels == plane_ind_2]
        # FIXME: Try efficient way of doing this
        for point1 in plane_1_points:
            for point2 in plane_2_points:
                distance = np.linalg.norm(point1 - point2)
                min_distances.append(distance)
        #min_dist = np.min(np.linalg.norm(plane_1_points - plane_2_points, axis=1)) # NOTE: Cannot use this : Not same dimension
        min_dist = np.min(np.asarray(min_distances))
        if min_dist <= smallest_dist and min_dist <= gripper_width: # NOTE: Gripper width checked while choosing final plane pair
            smallest_dist = min_dist
            plane_pairs = pair
        print(f"Minimum distance for plane pair {pair}: ", min_dist)
    
    # filtered_pairs = []
    # # Removing from plane pairs
    # for pair, min_dist in zip(plane_pairs, min_distances):
    #     if min_dist <= gripper_width:
    #         filtered_pairs.append(pair) # FIXME: Use pop instead

    # plane_pairs=filtered_pairs

    print("Filtered Plane pairs : {} with distance: {}".format(plane_pairs,smallest_dist))

    # Mean of each plane:
    plane1_mean = np.mean(pc_points[plane_labels == plane_pairs[0]], axis=0)
    plane2_mean = np.mean(pc_points[plane_labels == plane_pairs[1]], axis=0)

    # Compute the midpoint of the line connecting the two means
    midpoint = (plane1_mean + plane2_mean) / 2

    # Compute the gripper orientation
    gripper_normal = np.cross(plane1_mean - midpoint, plane2_mean - midpoint)
    gripper_theta = np.arctan2(gripper_normal[1], gripper_normal[0])

    # Update the gripper pose
    gripper_pose = np.array([midpoint[0], midpoint[1], midpoint[2], gripper_theta])
    print("Gripper pose : {}".format(gripper_pose))
    return gripper_pose


def main(n_tries=5):
    # Initialize the world
    world = helper.World()

    # start grasping loop
    # number of tries for grasping
    for i in range(n_tries):
        # get point cloud from cameras in the world
        pcd = world.get_point_cloud()
        # check point cloud to see if there are still objects to remove
        finish_flag = helper.check_pc(pcd)
        if finish_flag:  # if no more objects -- done!
            print('===============')
            print('Scene cleared')
            print('===============')
            break
        # visualize the point cloud from the scene
        helper.draw_pc(pcd)
        # compute antipodal grasp
        gripper_pose = get_antipodal(pcd)
        # send command to robot to execute
        robot_command = world.grasp(gripper_pose)
        # robot drops object to the side
        world.drop_in_bin(robot_command)
        # robot goes to initial configuration and prepares for next grasp
        world.home_arm()
        # go back to the top!

    # terminate simulation environment once you're done!
    p.disconnect()
    return finish_flag


if __name__ == "__main__":
    flag = main()
