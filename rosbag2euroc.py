#!/usr/bin/env python3

import argparse
import rosbag
import os
from std_msgs.msg import String

from src.camera import Camera
from src.imu import Imu
from src.lidar import Lidar
from src.transform import Transform
from src.odometry import Odometry
from src.utils import mkdirs_without_exception, sensor_file_strcuture_init




def rosbag_2_euroc(rosbag_path, output_path):
    # Check that the path to the rosbag exists.
    assert(os.path.exists(rosbag_path))
    bag = rosbag.Bag(rosbag_path)
    bag_metadata = bag.get_type_and_topic_info()
    camera_topics = []
    imu_topics = []
    tf_static_topics = []
    tf_topics = []
    lidar_topics = []
    gps_topics = []
    odometry_topics = []
    cmd_topics = []


    for key in bag_metadata.topics:

        if bag_metadata.topics[key].msg_type == 'sensor_msgs/Image':
            camera_topics.append(key)
        elif bag_metadata.topics[key].msg_type == 'sensor_msgs/Imu':
            imu_topics.append(key)
        elif bag_metadata.topics[key].msg_type in ['nav_msgs/Odometry', 'geometry_msgs/TransformStamped']:
            odometry_topics.append(key)
        elif bag_metadata.topics[key].msg_type == 'sensor_msgs/NavSatFix':
            gps_topics.append(key)
        elif bag_metadata.topics[key].msg_type == 'geometry_msgs/Twist':
            cmd_topics.append(key)
        elif bag_metadata.topics[key].msg_type == 'sensor_msgs/PointCloud2':
            lidar_topics.append(key)
        elif bag_metadata.topics[key].msg_type == 'tf2_msgs/TFMessage' and key == '/tf_static':
            tf_static_topics.append(key)
        elif bag_metadata.topics[key].msg_type == 'tf2_msgs/TFMessage' and key == '/tf':
            tf_topics.append(key)

    # Check that it has one or two Image topics.
    if not camera_topics:
        print ("WARNING: there are no camera topics in this rosbag!")

    # Check that it has one, and only one, Lidar topic.
    if lidar_topics != 1:
        print ("WARNING: expected to have a single Lidar topic, instead got: {} topic(s)".format(
                    len(lidar_topics)))

    # Check that it has one, and only one, IMU topic.
    if imu_topics != 1:
        print ("WARNING: expected to have a single IMU topic, instead got: {} topic(s)".format(
            len(imu_topics)))

    # Create base folder
    dirname = os.path.split(rosbag_path)[-1].split(".", 1)[0] + '/mav0'
    base_path = os.path.join(output_path, dirname)
    mkdirs_without_exception(base_path)


    # camera = Camera(base_path, camera_topics)
    # camera.convert(bag)

    # imu = Imu(base_path, imu_topics)
    # imu.convert(bag)

    # lidar = Lidar(base_path, lidar_topics)
    # lidar.convert(bag)

    static_tf = Transform(base_path, tf_static_topics)
    static_tf.convert(bag)

    tf = Transform(base_path, tf_topics)
    tf.convert(bag)

    
    
    # odometry = Odometry(base_path, odometry_topics)
    # odometry.convert(bag)


    # TODO(TONI): Consider adding Lidar msgs (sensor_msgs/PointCloud2)
    # TODO(TONI): parse tf_static or cam_info and create the sensor.yaml files?

    # Close the rosbag.
    bag.close()

if __name__ == "__main__":
    # Parse rosbag path.
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--rosbag_path', help='Path to the rosbag.', default='/media/zhipeng/zhipeng_usb/datasets/Orchard_EMR_Jun_2024_save_bags/from_20240603_120008_12_to_20240603_120459_99/merge_SlowRun_SlowBag_ZED2.bag')
    parser.add_argument('-o', '--output_path', help='Path to the output.', default='./')
    args = parser.parse_args()

    # Convert rosbag.
    print(f"Converting rosbag: \"{os.path.split(args.rosbag_path)[-1]}\" to EuRoC format.")
    print(f"Storing results in directory: {args.output_path}.")
    rosbag_2_euroc(args.rosbag_path, args.output_path)
    print("Done.")


