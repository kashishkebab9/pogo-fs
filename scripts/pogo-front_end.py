import bagpy # for reading rosbag
import pandas as pd # for reading csv
import networkx as nx # for pgo front end
import tf.transformations as tr
import numpy as np
import math
import matplotlib
import matplotlib.pyplot
import os
import open3d as o3d

pose_graph = nx.Graph()
bag = bagpy.bagreader('../bags/2023-01-26-15-26-20.bag')
print(bag.topic_table)
odom_motor_topic = "/scarab41/odom_motor"
cloud_topic = "/scarab41/laser_cloud"

odom_motor_csv = bag.message_by_topic(odom_motor_topic)
cloud_csv = bag.message_by_topic(cloud_topic)

odom_motor_data = pd.read_csv(odom_motor_csv)
cloud_data = pd.read_csv(cloud_csv)

directory = "../bags/pcd/"
list_of_pcds = []
for filename in os.listdir(directory):
    filename=filename[:-4]
    filename=float(filename)
    list_of_pcds.append(filename)

prior_pose = None
current_pose = None
prior_cloud = None
current_cloud = None

node_counter = 0
for index, row in odom_motor_data.iterrows():

    # time = row["Time"]
    quaternion_list = [row['pose.pose.orientation.x'],row['pose.pose.orientation.y'],row['pose.pose.orientation.z'],row['pose.pose.orientation.w']]
    quat = np.array(quaternion_list[:4], dtype=np.float64, copy=True)

    euler = tr.euler_from_quaternion(quat)
    pose = [row['pose.pose.position.x'], row['pose.pose.position.y'], euler[2]]
    if prior_pose == None:
        prior_pose = pose
        zero_time = row['Time']
        closest_time_dist = 100000
        pcd_index = None
        for i in range(len(list_of_pcds)):
            diff = abs(list_of_pcds[i] - zero_time)
            if diff < closest_time_dist:
                closest_time_dist = diff
                pcd_index = i
        prior_cloud = list_of_pcds[pcd_index]

        # find timestamp that closesly resembles and grab pcd file
        continue
    current_pose = pose
    current_time = row['Time']
    closest_time_dist = 100000
    pcd_index = None
    for i in range(len(list_of_pcds)):
        diff = abs(list_of_pcds[i] - current_time)
        if diff < closest_time_dist:
            closest_time_dist = diff
            pcd_index = i
    current_cloud = list_of_pcds[pcd_index]
    # grab the current timestamp closest pcd file

    if prior_pose != None and current_pose != None:
        # find the l2 norm, determine angle distance
        l2_norm = math.sqrt((current_pose[0] - prior_pose[0]) **2 + (current_pose[1] - prior_pose[1]) ** 2)

        # if current_pose[2] < 0:
        #     current_pose[2] = math.pi - current_pose[2]
        # if prior_pose[2] < 0:
        #     prior_pose[2] = math.pi - prior_pose[2]

        ang_diff = prior_pose[2] - current_pose[2] 

        if l2_norm > .5 or abs(ang_diff) > .5:
            print("prior_ang: ", prior_pose[2])
            print("current_ang: ", current_pose[2])
            print(ang_diff)
            motion_model_tf_matrix = np.array([[math.cos(ang_diff), -1*math.sin(ang_diff), 0, (prior_pose[0] - current_pose[0])],
                                               [math.sin(ang_diff), math.cos(ang_diff), 0, (prior_pose[1] - current_pose[1])],
                                               [0, 0, 1, 0],
                                               [0, 0, 0, 1]])
            print("motionmodel")
            print(motion_model_tf_matrix)
            # print(current_time)

            pose_graph.add_node(node_counter)
            if node_counter > 0:
                pose_graph.add_edge(node_counter - 1, node_counter)

            prior_cloud = str(prior_cloud)
            current_cloud = str(current_cloud)
            if len(prior_cloud) < 20:
                num_zeroes = 20 - len(str(prior_cloud))
                for i in range(num_zeroes):
                    prior_cloud += '0'
            if len(current_cloud) < 20:
                num_zeroes = 20 - len(str(current_cloud))
                for i in range(num_zeroes):
                    current_cloud += '0'

            prior_cloud_file = directory + str(prior_cloud) + ".pcd"
            current_cloud_file = directory + str(current_cloud) + ".pcd"

            prior_pcd = o3d.io.read_point_cloud(prior_cloud_file)
            current_pcd = o3d.io.read_point_cloud(current_cloud_file)
            prior_points = np.asarray(prior_pcd.points)
            current_points = np.asarray(current_pcd.points)

            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=1000)
            reg_result = o3d.pipelines.registration.registration_icp(
                    prior_pcd, current_pcd,  
                    max_correspondence_distance=2.00,  # Maximum correspondence distance
                    criteria=criteria
                    )
            transformation_matrix = reg_result.transformation
            if transformation_matrix[0, 3] < 0:
                motion_model_tf_matrix[0,3] = -1 * motion_model_tf_matrix[0,3]
                motion_model_tf_matrix[1,3] = -1 * motion_model_tf_matrix[1,3]
            print("icp transformation: ", transformation_matrix)

            w_better_init_result = o3d.pipelines.registration.registration_icp(
                    prior_pcd, current_pcd,  
                    max_correspondence_distance=2.00,  # Maximum correspondence distance
                    init=motion_model_tf_matrix,
                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
                    )
            init_transformation_matrix = w_better_init_result.transformation
            num_points_prior = len(prior_pcd.points)
            num_points_current = len(current_pcd.points)

            green_color = np.array([0.0, 1.0, 0.0])  # RGB values for green
            blue_color = np.array([0.0, 0.0, 1.0])  # RGB values for green
            red_color = np.array([1.0, 0.0, 1.0])  # RGB values for green
            black_color = np.array([0.0, 0.0, 0.0])  # RGB values for green
            # prior_pcd.colors = o3d.utility.Vector3dVector(np.tile(black_color, (num_points_prior, 1)))
            current_pcd.colors = o3d.utility.Vector3dVector(np.tile(blue_color, (num_points_current, 1)))


            with_better_init_prior = prior_pcd.transform(init_transformation_matrix)
            o3d.visualization.draw_geometries([with_better_init_prior, current_pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])

print("Generating Graph....")
fig = matplotlib.pyplot.figure()
nx.draw(pose_graph, ax=fig.add_subplot())
if True: 
    matplotlib.use("Agg") 
    fig.savefig("front-end.png")

## BACKEND


