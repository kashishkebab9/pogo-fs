import bagpy # for reading rosbag
import pandas as pd # for reading csv
import networkx # for pgo front end
#TODO: import pcl # for ICP
import tf.transformations as tr
import numpy as np

bag = bagpy.bagreader('../bags/2023-01-26-15-26-20.bag')

odom_motor_topic = "/scarab41/odom_motor"
odom_laser_topic = "/scarab41/odom_laser"

odom_motor_csv = bag.message_by_topic(odom_motor_topic)
odom_laser_csv = bag.message_by_topic(odom_laser_topic)

odom_motor_data = pd.read_csv(odom_motor_csv)
odom_laser_data = pd.read_csv(odom_laser_csv)

print(odom_motor_data)
print(odom_laser_data)

prior_pose = None
current_pose = None

for index, row in odom_motor_data.iterrows():
    if current_pose != None:
        prior_pose = current_pose

    # time = row["Time"]
    # print(row)
    quaternion_list = [row['pose.pose.orientation.x'],row['pose.pose.orientation.y'],row['pose.pose.orientation.z'],row['pose.pose.orientation.w']]
    quat = np.array(quaternion_list[:4], dtype=np.float64, copy=True)

    euler = tr.euler_from_quaternion(quat)
    pose = [row['pose.pose.position.x'], row['pose.pose.position.y'], euler[2]]
    current_pose = pose
    print(pose)
    if prior_pose != None:
        # find the l2 norm, determine angle distance



