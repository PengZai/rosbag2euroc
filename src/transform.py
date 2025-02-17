from .utils import mkdirs_without_exception, sensor_file_strcuture_init, ros_message_to_dict
import os
import json
from dataclasses import dataclass
import csv





class Transform(object):
    DATA_CSV = 'data.csv'
    DATA_CSV_HEADER = ["timestamp","p_RS_R_x [m]","p_RS_R_y [m]","p_RS_R_z [m]","q_RS_w []","q_RS_x []","q_RS_y []","q_RS_z []"]
    SENSOR_YAML_NAME = 'sensor.yaml'
    SENSOR_YAML = {
        "sensor_type": "transform",
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

        # Convert tf msg to Euroc dataset format.
        assert(len(self.topics) == len(self.folder_paths))
        for i, topic in enumerate(self.topics):
            print(f"Converting messages for topic: {topic}")
            print(f"Storing results in: {self.folder_paths[i]}")
            for topic, msg, t in bag.read_messages(topics=[topic]):
                for transform in msg.transforms:
                    transform_dict = ros_message_to_dict(transform)
                    timestamp_dict = transform_dict['header']['stamp']
                    transform_filename =  transform_dict['header']['frame_id'].replace('/', '_')+'_to_'+transform_dict['child_frame_id'].replace('/', '_')+'.csv'
                    
                    if not os.path.exists(os.path.join(self.folder_paths[i], transform_filename)):
                        with open(os.path.join(self.folder_paths[i], transform_filename), 'w+', newline='', encoding='utf-8') as outfile:
                            writer = csv.writer(outfile)
                            writer.writerow(self.DATA_CSV_HEADER)
                            
                    
                    with open(os.path.join(self.folder_paths[i], transform_filename), 'a', newline='', encoding='utf-8') as outfile:
                        writer = csv.writer(outfile)
                        #       timestamp               p_RS_R_x [m]	                   p_RS_R_y [m]	                      p_RS_R_z [m]	                     q_RS_w []	                     q_RS_x []	                     q_RS_y []	                     q_RS_z []	            
                        line = [transform.header.stamp, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]
                        writer.writerow(line) 

          