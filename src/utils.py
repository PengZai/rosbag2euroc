import os
import errno
import csv
import yaml
import cv2


def mkdirs_without_exception(path):
    try:
       os.makedirs(path)
    except OSError as e:
        if e.errno == errno.EEXIST:
            print("The directory {} already exists.".format(path))
        else:
            print(e)
            raise  # raises the error again

def ros_message_to_dict(msg):
    """
    Recursively convert a ROS message to a dictionary.
    Handles nested messages and arrays of messages.
    """
    if isinstance(msg, (int, float, str, bool)):
        return msg
    elif isinstance(msg, list):
        return [ros_message_to_dict(x) for x in msg]
    elif hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            result[slot] = ros_message_to_dict(value)
        return result
    else:
        # If the value is something we can't serialize directly, return as string
        return str(msg)


def sensor_file_strcuture_init(object, base_path, topics, data_csv_header):

    mkdirs_without_exception(base_path)
    # Create folder for camera topics
    folder_paths = []
    for i in range(len(topics)):
        folder_paths.append(os.path.join(base_path, topics[i].replace('/', '_')))
        latest_folder_path = folder_paths[-1]
        mkdirs_without_exception(latest_folder_path)
        # Create data folder
        mkdirs_without_exception(os.path.join(latest_folder_path, 'data'))
        # Create data.csv file
        with open(os.path.join(latest_folder_path, object.DATA_CSV), 'w+') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(data_csv_header)

        # Create sensor.yaml file
        with open(os.path.join(latest_folder_path, object.SENSOR_YAML_NAME), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            outfile.write("%YAML:1.0\n")
            object.SENSOR_YAML['comment'] = '_'.join(topics[i].split('/'))
            yaml.dump(object.SENSOR_YAML, outfile, default_flow_style=True)

    return folder_paths