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

for index, row in odom_motor_data.iterrows():
    # time = row["Time"]
    # print(row)
    quaternion_list = [row['pose.pose.orientation.x'],row['pose.pose.orientation.y'],row['pose.pose.orientation.z'],row['pose.pose.orientation.w']]
    quat = np.array(quaternion_list[:4], dtype=np.float64, copy=True)

    euler = tr.euler_from_quaternion(quat)
    print(quaternion_list)
    print(euler)
    # print(time)
