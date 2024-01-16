import bagpy # for reading rosbag
import pandas as pd # for reading csv
import networkx as nx # for pgo front end
import tf.transformations as tr
import numpy as np
import math
import matplotlib
import matplotlib.pyplot

bag = bagpy.bagreader('../bags/2023-01-26-15-26-20.bag')
pose_graph = nx.Graph()
print(bag.topic_table)

odom_motor_topic = "/scarab41/odom_motor"
odom_laser_topic = "/scarab41/odom_laser"

odom_motor_csv = bag.message_by_topic(odom_motor_topic)
odom_laser_csv = bag.message_by_topic(odom_laser_topic)

odom_motor_data = pd.read_csv(odom_motor_csv)
odom_laser_data = pd.read_csv(odom_laser_csv)

# print(odom_motor_data)
# print(odom_laser_data)

prior_pose = None
current_pose = None

node_counter = 0
for index, row in odom_motor_data.iterrows():

    # time = row["Time"]
    # print(row)
    quaternion_list = [row['pose.pose.orientation.x'],row['pose.pose.orientation.y'],row['pose.pose.orientation.z'],row['pose.pose.orientation.w']]
    quat = np.array(quaternion_list[:4], dtype=np.float64, copy=True)

    euler = tr.euler_from_quaternion(quat)
    pose = [row['pose.pose.position.x'], row['pose.pose.position.y'], euler[2]]
    if prior_pose == None:
        prior_pose = pose
        continue
    current_pose = pose
    print(pose)
    print(current_pose)
    print(prior_pose)

    if prior_pose != None and current_pose != None:
        # find the l2 norm, determine angle distance
        l2_norm = math.sqrt((current_pose[0] - prior_pose[0]) **2 + (current_pose[1] - prior_pose[1]) ** 2)
        if current_pose[2] < 0:
            current_pose[2] = math.pi - current_pose[2]
        if prior_pose[2] < 0:
            prior_pose[2] = math.pi - prior_pose[2]
        ang_diff = abs(prior_pose[2] - current_pose[2])

        if l2_norm > .5 or ang_diff > .5:
            print(l2_norm)
            print(ang_diff)
            print("Adding node and edge")
            pose_graph.add_node(node_counter)
            if node_counter > 0:
                pose_graph.add_edge(node_counter - 1, node_counter)
            prior_pose = current_pose

            node_counter+=1

print("Generating Graph....")
fig = matplotlib.pyplot.figure()
nx.draw(pose_graph, ax=fig.add_subplot())
if True: 
    # Save plot to file
    matplotlib.use("Agg") 
    fig.savefig("graph.png")
else:
    # Display interactive viewer
    matplotlib.pyplot.show()


        
