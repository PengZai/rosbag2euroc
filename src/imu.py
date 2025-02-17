from .utils import mkdirs_without_exception, sensor_file_strcuture_init
import os
import csv


class Imu(object):
    DATA_CSV = 'data.csv'
    DATA_CSV_HEADER = ["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]"]
    SENSOR_YAML_NAME = 'sensor.yaml'
    SENSOR_YAML = {
        "sensor_type": "imu",
        "comment": "VI-Sensor IMU (ADIS16448)",
        "T_BS": {
            "cols": 4,
            "rows": 4,
            "data": [1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0]
        },
        "rate_hz": 200,
        "gyroscope_noise_density": 1.6968e-04,
        "gyroscope_random_walk": 1.9393e-05,
        "accelerometer_noise_density": 2.0000e-3,
        "accelerometer_random_walk": 3.0000e-3
    }

    def __init__(self, base_path, topics):


        self.base_path = base_path
        self.topics = topics


        self.folder_paths = sensor_file_strcuture_init(self, base_path, topics, self.DATA_CSV_HEADER)


    

    def convert(self, bag):

        # Convert IMU msg to Euroc dataset format.
        assert(len(self.topics) == len(self.folder_paths))
        for i, topic in enumerate(self.topics):
            print(f"Converting messages for topic: {topic}")
            print(f"Storing results in: {self.folder_paths[i]}")
            with open(os.path.join(self.folder_paths[i], self.DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
                for _, msg, t in bag.read_messages(topics=[topic]):
                    writer = csv.writer(outfile)
                    line = [msg.header.stamp, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
                    writer.writerow(line)  
                  

