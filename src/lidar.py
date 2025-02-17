from .utils import mkdirs_without_exception, sensor_file_strcuture_init
import os
import csv
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d


class Lidar(object):
    DATA_CSV = 'data.csv'
    DATA_CSV_HEADER = ["#timestamp [ns]","filename"]
    SENSOR_YAML_NAME = 'sensor.yaml'
    SENSOR_YAML = {
        "sensor_type": "lidar",
        "comment": "right now just a place holder for further develop",
        "T_BS": {
                "cols": 4,
                "rows": 4,
                "data": [1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0]
            }
    }


    def __init__(self, base_path, topics):


        self.base_path = base_path
        self.topics = topics
        self.folder_paths = sensor_file_strcuture_init(self, base_path, topics, self.DATA_CSV_HEADER)


    

    def convert(self, bag):

        # Convert Lidar msg to Euroc dataset format.
        assert(len(self.topics) == len(self.folder_paths))
        for i, topic in enumerate(self.topics):
            print(f"Converting messages for topic: {topic}")
            print(f"Storing results in: {self.folder_paths[i]}")

            with open(os.path.join(self.folder_paths[i], self.DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
                for topic, msg, t in bag.read_messages(topics=[topic]):

                    nsec_timestamp = str(msg.header.stamp.to_nsec())
                    lidar_filename = nsec_timestamp[:10]+'.'+nsec_timestamp[10:] + '.pcd'

                    save_path = os.path.join(self.folder_paths[i], 'data/', lidar_filename)
                    if not os.path.exists(save_path):
                        print(f"writing {save_path}")
                        writer = csv.writer(outfile)
                        line = [msg.header.stamp, lidar_filename]
                        writer.writerow(line) 

                        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
                        # Convert to Open3D point cloud
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(points)
                        o3d.io.write_point_cloud(save_path, pcd)
                    else:
                        print(f"{save_path} has existed")

               

