from .utils import mkdirs_without_exception, sensor_file_strcuture_init
import os
import csv


class Odometry(object):
    DATA_CSV = 'data.csv'
    DATA_CSV_HEADER = ["#timestamp", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]", "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []", "v_RS_R_x [m s^-1]", "v_RS_R_y [m s^-1]", "v_RS_R_z [m s^-1]", "b_w_RS_S_x [rad s^-1]", "b_w_RS_S_y [rad s^-1]",	 "b_w_RS_S_z [rad s^-1]", "b_a_RS_S_x [m s^-2]", "b_a_RS_S_y [m s^-2]", "b_a_RS_S_z [m s^-2]"]
    SENSOR_YAML_NAME = 'sensor.yaml'
    SENSOR_YAML = {
        "sensor_type": "odometry",
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

        # Convert Odometry msg to Euroc dataset format.
        assert(len(self.topics) == len(self.folder_paths))
        for i, topic in enumerate(self.topics):
            print(f"Converting messages for topic: {topic}")
            print(f"Storing results in: {self.folder_paths[i]}")
            with open(os.path.join(self.folder_paths[i], self.DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
                for _, msg, t in bag.read_messages(topics=[topic]):
                    writer = csv.writer(outfile)
                    #       timestamp            p_RS_R_x [m]	              p_RS_R_y [m]	              p_RS_R_z [m]	               q_RS_w []	               q_RS_x []	             q_RS_y []	                      q_RS_z []	            v_RS_R_x [m s^-1]	       v_RS_R_y [m s^-1]	   v_RS_R_z [m s^-1]	     b_w_RS_S_x [rad s^-1]	    b_w_RS_S_y [rad s^-1]	  b_w_RS_S_z [rad s^-1]	   b_a_RS_S_x [m s^-2]	  b_a_RS_S_y [m s^-2]	 b_a_RS_S_z [m s^-2]
                    line = [msg.header.stamp, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, 0,                     0,                     0]
                    writer.writerow(line)  

