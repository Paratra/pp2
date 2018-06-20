import os
import rosbag
import numpy as np

import sensor_msgs.point_cloud2 as pc2

# zmin = -1.0
# zmas = 0.2

files = os.listdir('/home/ming/pp2/data/ikg-hddmap/')
#print (files)

pointclouds = []

bag = rosbag.Bag(os.path.join('/home/ming/pp2/data/ikg-hddmap/','only-gps-velodyne.bag'))
#print (bag)
print (bag.read_messages(
    topics = [
        '/velodyne_packges'
    ]))
for data in bag.read_messages(
    topics = [
        '/velodyne_packges'
    ]):
    lidar = pc2.read_points(msg)
    points = np.array(list(lidar))
    #points = points[np.where([zmin <= point[2] <= zmax for point in points])]
    print '123'
    pointcloud.append(points)
