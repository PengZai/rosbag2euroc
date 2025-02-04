#!/usr/bin/env python3

import sys
import argparse
import os
import subprocess, yaml
import errno

import cv2

import roslib
import rosbag
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import csv
import numpy as np
import sensor_msgs
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import json





CAM_FOLDER_NAME = 'cam'
IMU_FOLDER_NAME = 'imu'
LIDAR_FOLDER_NAME = 'lidar'
TF_STATIC_FOLDER_NAME = 'tf_static'
TF_FOLDER_NAME = 'tf'
GT_FOLDER_NAME = 'state_groundtruth_estimate'
DATA_CSV = 'data.csv'
SENSOR_YAML = 'sensor.yaml'
BODY_YAML = 'body.yaml'

# Can get this from ros topic
CAM_SENSOR_YAML = {
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

# Can get this from ros topic
IMU_SENSOR_YAML = {
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


# Can get this from ros topic
STATE_GROUNDTRUTH_ESTIMATE_YAML = {
    "sensor_type": "visual-inertial",
    "comment": "The nonlinear least-squares batch solution over the Leica position and IMU measurements including time offset estimation. The orientation is only observed by the IMU.",
    "T_BS": {
        "cols": 4,
        "rows": 4,
        "data": [1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0]
    }
}


LIDAR_SENSOR_YAML = {
    "sensor_type": "visual-inertial",
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



def get_rosbag_metadata(rosbag_path):
    assert(os.path.exists(rosbag_path))
    # This subprocess will only work if ROS is sourced...
    return yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_path],
                                      stdout=subprocess.PIPE).communicate()[0], Loader=yaml.SafeLoader)

def mkdirs_without_exception(path):
    try:
       os.makedirs(path)
    except OSError as e:
        if e.errno == errno.EEXIST:
            print("The directory {} already exists.".format(path))
        else:
            print(e)
            raise  # raises the error again

def setup_dataset_dirs(rosbag_path, output_path, camera_topics, imu_topics, lidar_topics, tf_static_topics, tf_topics, gt_topics):
    # Create base folder
    dirname = os.path.split(rosbag_path)[-1].split(".", 1)[0] + '/mav0'
    base_path = os.path.join(output_path, dirname)
    mkdirs_without_exception(base_path)

    # Create folder for camera topics
    cam_folder_paths = []
    for i in range(len(camera_topics)):
        cam_folder_paths.append(os.path.join(base_path, CAM_FOLDER_NAME + repr(i)))
        current_cam_folder_path = cam_folder_paths[-1]
        mkdirs_without_exception(current_cam_folder_path)

        # Create data folder
        mkdirs_without_exception(os.path.join(current_cam_folder_path, 'data'))

        # Create data.csv file
        with open(os.path.join(current_cam_folder_path, DATA_CSV), 'w+') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(["#timestamp [ns]","filename"])

        # Create sensor.yaml file
        with open(os.path.join(current_cam_folder_path, SENSOR_YAML), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            outfile.write("%YAML:1.0\n")
            CAM_SENSOR_YAML['comment'] = CAM_FOLDER_NAME + repr(i)
            yaml.dump(CAM_SENSOR_YAML, outfile, default_flow_style=True)


    # Create folder for imu topics
    imu_folder_paths = []
    for i in range(len(imu_topics)):
        imu_folder_paths.append(os.path.join(base_path, IMU_FOLDER_NAME + repr(i)))
        current_imu_folder_path = imu_folder_paths[-1]
        mkdirs_without_exception(current_imu_folder_path)

        # Create data.csv file
        with open(os.path.join(current_imu_folder_path, DATA_CSV), 'w+') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]"])
            #timestamp [ns]	w_RS_S_x [rad s^-1]	w_RS_S_y [rad s^-1]	w_RS_S_z [rad s^-1]	a_RS_S_x [m s^-2]	a_RS_S_y [m s^-2]	a_RS_S_z [m s^-2]

        # Create sensor.yaml file
        with open(os.path.join(current_imu_folder_path, SENSOR_YAML), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            outfile.write("%YAML:1.0\n")
            IMU_SENSOR_YAML['comment'] = IMU_FOLDER_NAME + repr(i)
            yaml.dump(IMU_SENSOR_YAML, outfile, default_flow_style=True)


    # Create folder for tf topics
    tf_static_folder_paths = []
    for i in range(len(tf_static_topics)):
        tf_static_folder_paths.append(os.path.join(base_path, TF_STATIC_FOLDER_NAME + repr(i)))
        current_tf_static_folder_path = tf_static_folder_paths[-1]
        mkdirs_without_exception(current_tf_static_folder_path)

        # Create data folder
        mkdirs_without_exception(os.path.join(current_tf_static_folder_path, 'data'))

        # Create data.csv file
        with open(os.path.join(current_tf_static_folder_path, DATA_CSV), 'w+') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(["#timestamp [ns]","filename"])  


    # Create folder for tf topics
    tf_folder_paths = []
    for i in range(len(tf_topics)):
        tf_folder_paths.append(os.path.join(base_path, TF_FOLDER_NAME + repr(i)))
        current_tf_folder_path = tf_folder_paths[-1]
        mkdirs_without_exception(current_tf_folder_path)

        # Create data folder
        mkdirs_without_exception(os.path.join(current_tf_folder_path, 'data'))

        # Create data.csv file
        with open(os.path.join(current_tf_folder_path, DATA_CSV), 'w+') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(["#timestamp [ns]","filename"])  


    # Create folder for lidar topics
    lidar_folder_paths = []
    for i in range(len(lidar_topics)):
        lidar_folder_paths.append(os.path.join(base_path, LIDAR_FOLDER_NAME + repr(i)))
        current_lidar_folder_path = lidar_folder_paths[-1]

        mkdirs_without_exception(current_lidar_folder_path)

        # Create data folder
        mkdirs_without_exception(os.path.join(current_lidar_folder_path, 'data'))

        

        # Create data.csv file
        with open(os.path.join(current_lidar_folder_path, DATA_CSV), 'w+') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(["#timestamp [ns]","filename"])  

        # Create sensor.yaml file
        with open(os.path.join(current_lidar_folder_path, SENSOR_YAML), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            outfile.write("%YAML:1.0\n")
            LIDAR_SENSOR_YAML['comment'] = LIDAR_FOLDER_NAME + repr(i)
            yaml.dump(LIDAR_SENSOR_YAML, outfile, default_flow_style=True)

    # Create folder for ground truth topics
    gt_folder_paths = []
    for i in range(len(gt_topics)):
        gt_folder_paths.append(os.path.join(base_path, GT_FOLDER_NAME + repr(i)))
        current_gt_folder_path = gt_folder_paths[-1]
        mkdirs_without_exception(current_gt_folder_path)
        

        # Create data.csv file
        with open(os.path.join(current_gt_folder_path, DATA_CSV), 'w+') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(["#timestamp", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]", "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []", "v_RS_R_x [m s^-1]", "v_RS_R_y [m s^-1]", "v_RS_R_z [m s^-1]", "b_w_RS_S_x [rad s^-1]", "b_w_RS_S_y [rad s^-1]",	 "b_w_RS_S_z [rad s^-1]", "b_a_RS_S_x [m s^-2]", "b_a_RS_S_y [m s^-2]", "b_a_RS_S_z [m s^-2]"])
                              #timestamp    p_RS_R_x [m]	p_RS_R_y [m]	p_RS_R_z [m]	q_RS_w []	 q_RS_x []	  q_RS_y []	   q_RS_z []	v_RS_R_x [m s^-1]	 v_RS_R_y [m s^-1]	  v_RS_R_z [m s^-1]	   b_w_RS_S_x [rad s^-1]	b_w_RS_S_y [rad s^-1]	  b_w_RS_S_z [rad s^-1]	   b_a_RS_S_x [m s^-2]	  b_a_RS_S_y [m s^-2]	 b_a_RS_S_z [m s^-2]

        # Create sensor.yaml file
        with open(os.path.join(current_gt_folder_path, SENSOR_YAML), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            outfile.write("%YAML:1.0\n")
            STATE_GROUNDTRUTH_ESTIMATE_YAML['comment'] = GT_FOLDER_NAME + repr(i)
            yaml.dump(STATE_GROUNDTRUTH_ESTIMATE_YAML, outfile, default_flow_style=True)

    

    
    # Create body.yaml file
    with open(os.path.join(base_path, BODY_YAML), 'w+') as outfile:
        outfile.write("%YAML:1.0\n")
        body_yaml = dict(comment = 'Automatically generated dataset using Rosbag2Euroc, using rosbag: {}'.format(rosbag_path))
        yaml.dump(body_yaml, outfile, default_flow_style=True)

    return cam_folder_paths, imu_folder_paths, lidar_folder_paths, tf_static_folder_paths, tf_folder_paths, gt_folder_paths

def rosbag_2_euroc(rosbag_path, output_path):
    # Check that the path to the rosbag exists.
    assert(os.path.exists(rosbag_path))
    bag = rosbag.Bag(rosbag_path)

    # Check that rosbag has the data we need to convert to Euroc dataset format.
    bag_metadata = get_rosbag_metadata(rosbag_path)
    # sort topics with number of message
    bag_metadata['topics'] = sorted(bag_metadata['topics'], key=lambda x: x["messages"], reverse=True)
    camera_topics = []
    imu_topics = []
    tf_static_topics = []
    tf_topics = []
    lidar_topics = []
    gt_topics = []
    for element in bag_metadata['topics']:
        if (element['type'] == 'sensor_msgs/Image'):
            camera_topics.append(element['topic'])
        elif (element['type'] == 'sensor_msgs/Imu'):
            imu_topics.append(element['topic'])
        elif (element['type'] == 'nav_msgs/Odometry'):
            gt_topics.append(element['topic'])
        elif (element['type'] == 'sensor_msgs/PointCloud2'):
            lidar_topics.append(element['topic'])
        elif (element['type'] == 'tf2_msgs/TFMessage' and element['topic'] == '/tf_static'):
            tf_static_topics.append(element['topic'])
        elif (element['type'] == 'tf2_msgs/TFMessage' and element['topic'] == '/tf'):
            tf_topics.append(element['topic'])

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

    # Build output folder.
    cam_folder_paths, imu_folder_paths, lidar_folder_paths, tf_static_folder_paths, tf_folder_paths, gt_folder_paths = setup_dataset_dirs(rosbag_path, output_path, camera_topics, imu_topics, lidar_topics, tf_static_topics, tf_topics, gt_topics)

    # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
    cv_bridge = CvBridge()

    # Convert image msg to Euroc dataset format.
    assert(len(camera_topics) == len(cam_folder_paths))
    for i, cam_topic in enumerate(camera_topics):
        print(f"Converting camera messages for topic: {cam_topic}")
        print(f"Storing results in: {cam_folder_paths[i]}")
        # Write data.csv file.
        with open(os.path.join(cam_folder_paths[i], DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
            for topic, msg, t in bag.read_messages(topics=[cam_topic]):

                nsec_timestamp = str(msg.header.stamp.to_nsec())
                image_filename = nsec_timestamp[:10]+'.'+nsec_timestamp[10:] + '.png'
                writer = csv.writer(outfile)
                line = [msg.header.stamp, image_filename]
                writer.writerow(line) 
                try:
                    save_path = os.path.join(cam_folder_paths[i], 'data/', image_filename)
                    if not os.path.exists(save_path):
                        print(f"writing {save_path}")
                        cv_image = cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                        cv2.imwrite(save_path, cv_image)
                    else:
                        print(f"{save_path} has existed")

                except CvBridgeError as e:
                    print(e)

    # Convert IMU msg to Euroc dataset format.
    assert(len(imu_topics) == len(imu_folder_paths))
    for i, imu_topic in enumerate(imu_topics):
        print(f"Converting IMU messages for topic: {imu_topic}")
        print(f"Storing results in: {imu_folder_paths[i]}")
        with open(os.path.join(imu_folder_paths[i], DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
            for _, msg, t in bag.read_messages(topics=[imu_topic]):
                writer = csv.writer(outfile)
                line = [msg.header.stamp, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
                writer.writerow(line)  

    # Convert Lidar msg to Euroc dataset format.
    assert(len(lidar_topics) == len(lidar_folder_paths))
    for i, lidar_topic in enumerate(lidar_topics):
        print(f"Converting LIDAR messages for topic: {lidar_topic}")
        print(f"Storing results in: {lidar_folder_paths[i]}")

        with open(os.path.join(lidar_folder_paths[i], DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
            for topic, msg, t in bag.read_messages(topics=[lidar_topic]):

                nsec_timestamp = str(msg.header.stamp.to_nsec())
                lidar_filename = nsec_timestamp[:10]+'.'+nsec_timestamp[10:] + '.pcd'

                save_path = os.path.join(lidar_folder_paths[i], 'data/', lidar_filename)
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


    # Convert tf msg to Euroc dataset format.
    assert(len(tf_static_topics) == len(tf_static_folder_paths))
    for i, tf_static_topic in enumerate(tf_static_topics):
        print(f"Converting TF static messages for topic: {tf_static_topic}")
        print(f"Storing results in: {tf_static_folder_paths[i]}")
        with open(os.path.join(tf_static_folder_paths[i], DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
            for topic, msg, t in bag.read_messages(topics=[tf_static_topic]):
                for transform in msg.transforms:
                    nsec_timestamp = str(transform.header.stamp.to_nsec())
                    transform_filename = nsec_timestamp[:10]+'.'+nsec_timestamp[10:] + '.json'
                    transform_dict = {
                        "header": {
                            "stamp": str(transform.header.stamp),
                            "frame_id": transform.header.frame_id,
                        },
                        "child_frame_id": transform.child_frame_id,
                        "transform": {
                            "translation": {
                                "x": transform.transform.translation.x,
                                "y": transform.transform.translation.y,
                                "z": transform.transform.translation.z,
                            },
                            "rotation": {
                                "x": transform.transform.rotation.x,
                                "y": transform.transform.rotation.y,
                                "z": transform.transform.rotation.z,
                                "w": transform.transform.rotation.w,
                            },
                        },
                    }
                    
                    with open(os.path.join(tf_static_folder_paths[i], 'data/', transform_filename), "w") as f:
                        json.dump(transform_dict, f, indent=4)

    # Convert tf msg to Euroc dataset format.
    assert(len(tf_topics) == len(tf_folder_paths))
    for i, tf_topic in enumerate(tf_topics):
        print(f"Converting TF messages for topic: {tf_topic}")
        print(f"Storing results in: {tf_folder_paths[i]}")
        with open(os.path.join(tf_folder_paths[i], DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
            for topic, msg, t in bag.read_messages(topics=[tf_topic]):
                for transform in msg.transforms:
                    nsec_timestamp = str(transform.header.stamp.to_nsec())
                    transform_filename = nsec_timestamp[:10]+'.'+nsec_timestamp[10:] + '.json'
                    transform_dict = {
                        "header": {
                            "stamp": str(transform.header.stamp),
                            "frame_id": transform.header.frame_id,
                        },
                        "child_frame_id": transform.child_frame_id,
                        "transform": {
                            "translation": {
                                "x": transform.transform.translation.x,
                                "y": transform.transform.translation.y,
                                "z": transform.transform.translation.z,
                            },
                            "rotation": {
                                "x": transform.transform.rotation.x,
                                "y": transform.transform.rotation.y,
                                "z": transform.transform.rotation.z,
                                "w": transform.transform.rotation.w,
                            },
                        },
                    }
                    with open(os.path.join(os.path.join(tf_folder_paths[i], 'data/', transform_filename), "w")) as f:
                        json.dump(transform_dict, f, indent=4)
                
                



    # Convert GT msg to Euroc dataset format.
    assert(len(gt_topics) == len(gt_folder_paths))
    for i, gt_topic in enumerate(gt_topics):
        print(f"Converting GT messages for topic: {gt_topic}")
        print(f"Storing results in: {gt_folder_paths[i]}")
        with open(os.path.join(gt_folder_paths[i], DATA_CSV), 'a', newline='', encoding='utf-8') as outfile:
            for _, msg, t in bag.read_messages(topics=[gt_topic]):
                writer = csv.writer(outfile)
                #       timestamp            p_RS_R_x [m]	              p_RS_R_y [m]	              p_RS_R_z [m]	               q_RS_w []	               q_RS_x []	             q_RS_y []	                      q_RS_z []	            v_RS_R_x [m s^-1]	       v_RS_R_y [m s^-1]	   v_RS_R_z [m s^-1]	     b_w_RS_S_x [rad s^-1]	    b_w_RS_S_y [rad s^-1]	  b_w_RS_S_z [rad s^-1]	   b_a_RS_S_x [m s^-2]	  b_a_RS_S_y [m s^-2]	 b_a_RS_S_z [m s^-2]
                line = [msg.header.stamp, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, 0,                     0,                     0]
                writer.writerow(line)  


    # TODO(TONI): Consider adding Lidar msgs (sensor_msgs/PointCloud2)
    # TODO(TONI): parse tf_static or cam_info and create the sensor.yaml files?

    # Close the rosbag.
    bag.close()

if __name__ == "__main__":
    # Parse rosbag path.
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--rosbag_path', help='Path to the rosbag.', default='/media/hpc_ubuntu/zhipeng_usb/Calibration_theta_ouster.bag')
    parser.add_argument('-o', '--output_path', help='Path to the output.', default='./')
    args = parser.parse_args()

    # Convert rosbag.
    print(f"Converting rosbag: \"{os.path.split(args.rosbag_path)[-1]}\" to EuRoC format.")
    print(f"Storing results in directory: {args.output_path}.")
    rosbag_2_euroc(args.rosbag_path, args.output_path)
    print("Done.")


