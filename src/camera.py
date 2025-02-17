from .utils import mkdirs_without_exception, sensor_file_strcuture_init
import os
import csv
import cv2
from cv_bridge import CvBridge, CvBridgeError




class Camera(object):
    DATA_CSV = 'data.csv'
    DATA_CSV_HEADER = ["#timestamp [ns]","filename"]
    SENSOR_YAML_NAME = 'sensor.yaml'
    SENSOR_YAML = {
            "sensor_type" : "camera",
            "comment" : "VI-Sensor cam0 (MT9M034)",
            "T_BS": {
                "cols": 4,
                "rows": 4,
                "data": [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                    0.0, 0.0, 0.0, 1.0]
            },
            "rate_hz": 20,
            "resolution": [752, 480],
            "camera_model": "pinhole",
            "intrinsics": [458.654, 457.296, 367.215, 248.375],
            "distortion_model": "radial-tangential",
            "distortion_coefficients": [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
    }

    def __init__(self, base_path, topics):


        self.base_path = base_path
        self.topics = topics
        self.cv_bridge = CvBridge()


        self.folder_paths = sensor_file_strcuture_init(self, base_path, topics, self.DATA_CSV_HEADER)

    def convert(self, bag):

        # Convert image msg to Euroc dataset format.
        assert(len(self.topics) == len(self.folder_paths))
        for i, topic in enumerate(self.topics):
            print(f"Converting messages for topic: {topic}")
            print(f"Storing results in: {self.folder_paths[i]}")
            # Write data.csv file.
            with open(os.path.join(self.folder_paths[i], self.DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
                for topic, msg, t in bag.read_messages(topics=[topic]):

                    nsec_timestamp = str(msg.header.stamp.to_nsec())
                    image_filename = nsec_timestamp[:10]+'.'+nsec_timestamp[10:] + '.png'
                    writer = csv.writer(outfile)
                    line = [msg.header.stamp, image_filename]
                    writer.writerow(line) 
                    try:
                        save_path = os.path.join(self.folder_paths[i], 'data/', image_filename)
                        if not os.path.exists(save_path):
                            print(f"writing {save_path}")
                            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                            cv2.imwrite(save_path, cv_image)
                        else:
                            print(f"{save_path} has existed")

                    except CvBridgeError as e:
                        print(e)

                 

