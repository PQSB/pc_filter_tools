import os
import sys
import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from utils import *

from tqdm import tqdm

def export_msgs(bag_path, topics2export):
    # Configure reader
    if os.path.isdir(bag_path):
        files = os.listdir(bag_path)
        if any(f.endswith('.mcap') for f in files):
            storage_id = 'mcap'
        else:
            storage_id = 'sqlite3'
    else:
        sys.exit(f"Error: Invalid path, not a directory: {bag_path}")

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id=storage_id
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    for key, info in topics2export.items():
        user_topic = info["topic"]
        if user_topic not in type_map:
            print(f"❌ ERROR: The topic'{user_topic}' is not in rosbag")
            return

    pbar = tqdm(
        desc="Exporting messages",
        dynamic_ncols=True,
        total=None,
        mininterval=0.2,
        unit="msg"
    )

    while reader.has_next():
        topic, data, t = reader.read_next()
        pbar.update()

        if topic not in [info["topic"] for info in topics2export.values()]:
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        for key, info in topics2export.items():
            if topic == info["topic"]:
                if key == "lidar":
                    export_cloud(msg, info["out_path"])

                elif key == "images":
                    export_image(msg, info["out_path"])

                elif key == "depth_img":
                    export_depth_image(msg, info["out_path"])

                elif key == "depth2cloud":
                    export_cloud_from_depth(msg, info["k"], info["out_path"])

def parse_topic_and_dir(param):
    """
    Splits the topic and the directory names
    """
    if ":" not in param:
        raise ValueError(f"Invalid format: '{param}'. Required topic:directory_name")

    topic, directory = param.split(":", 1)

    topic = topic.strip()
    directory = directory.strip()

    if not topic or not directory:
        raise ValueError(f"Invalid format: '{param}'. Required topic:directory_name")

    return topic, directory


def prepare_args(lidar, images, depth_img, depth2cloud, k_depth, outdir):
    dict = {}

    if lidar:
        topic, directory = parse_topic_and_dir(lidar)
        dict["lidar"] = {
            "topic": topic,
            "dir": directory
        }

    if images:
        topic, directory = parse_topic_and_dir(images)
        dict["images"] = {
            "topic": topic,
            "dir": directory
        }

    if depth_img:
        topic, directory = parse_topic_and_dir(depth_img)
        dict["depth_img"] = {
            "topic":topic,
            "dir": directory
        }

    if depth2cloud:
        topic, directory = parse_topic_and_dir(depth2cloud)
        dict["depth2cloud"] = {
            "topic": topic,
            "dir": directory,
            "k": load_intrinsics_from_yaml(k_depth)
        }

    # Create the final path to the output directory of every topic
    for key, info in dict.items():
        out_dir = os.path.join(outdir, info["dir"])
        os.makedirs(out_dir, exist_ok=True)
        info["out_path"] = out_dir

    return dict

def main():
    parser = argparse.ArgumentParser(description="Export the lidar, camera, and depth image point cloud data to subdirectories within the main directory")
    parser.add_argument("--bag_path", type=str, required=True, help="Rosbag path")

    parser.add_argument("--out_root_dir", type=str, required=True, help="Root directory in which to create the subdirectories")
    parser.add_argument("--lidar", type=str, help="Provide the lidar topic and the directory name /topic:directory_name")
    parser.add_argument("--depth_img", type=str, help="Provide the depth images topic and the directory name /topic:directory_name")
    parser.add_argument("--images", type=str, help="Provide the images topic and the directory name /topic:directory_name")
    parser.add_argument("--depth2cloud", type=str, help="Provide the depth images topic and the name of the directory to which the point cloud generated from the depth image should be exported /topic:directory_name")
    parser.add_argument("--depth_k", type=str, help="Yaml file with the K matrix needed to obtain the point cloud from the depth image")

    args = parser.parse_args()

    if args.depth2cloud and not args.depth_k:
        print("ERROR: If --depth2cloud is used, argument --depth_k must be used too")
        sys.exit(1)

    # Check if out_root_dir exists and create it in case it doesn't
    if not os.path.exists(args.out_root_dir):
        print(f"Directory {args.out_root_dir} doesn't exists. Creating it...")
        os.makedirs(args.out_root_dir, exist_ok=True)

    topics2export = prepare_args(
        args.lidar, args.images, args.depth_img, args.depth2cloud, args.depth_k, args.out_root_dir)

    export_msgs(args.bag_path, topics2export)

if __name__ == "__main__":
    main()
