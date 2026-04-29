import argparse
import bisect
import os
import csv
from tqdm import tqdm

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def store_topics_times(path, b_topic, t_topic):
    # Configuración del reader
    storage_options = rosbag2_py.StorageOptions(
        uri=path,
        storage_id='mcap'
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Comprobar que los topics existen en el rosbag
    if b_topic not in type_map:
        print(f"❌ Topic: {b_topic} doesn't exist")
        return
    if t_topic not in type_map:
        print(f"❌ Topic: {t_topic} doesn't exist")
        return

    target_msgs = []
    base_msgs = []

    target_msg_type = get_message(type_map[t_topic])
    base_msg_type = get_message(type_map[b_topic])

    cam_idx = 0
    lidar_idx = 0

    pbar = tqdm(desc="Reading rosbag", dynamic_ncols=True)

    # Recorrer el rosbag guardando en un array los mensajes de cada topic
    while reader.has_next():
        topic, data, t = reader.read_next()
        pbar.update(1)

        if topic == b_topic:
            msg = deserialize_message(data, base_msg_type)
            time = (msg.header.stamp.sec * 1_000_000_000) + msg.header.stamp.nanosec
            
            base_msgs.append((time, lidar_idx))
            lidar_idx += 1

        elif topic == t_topic:
            msg = deserialize_message(data, target_msg_type)
            time = (msg.header.stamp.sec * 1_000_000_000) + msg.header.stamp.nanosec
            
            target_msgs.append((time, cam_idx))
            cam_idx += 1

    return base_msgs, target_msgs


def sync_base_target(path, b_topic, t_topic):
    base_times, target_times = store_topics_times(path, b_topic, t_topic)

    # buscar donde iria ubicado la medida del lidar en las medidas de la camara
    # y comparar con la anterior y posterior para deterinar la mejor
    cam_ts = [ts for (ts, _) in target_times]
    lidar_ts = [ts for (ts, _) in base_times]

    pairs = []

    for i in tqdm(range(len(lidar_ts)), desc="Synchronizing topics"):
        pos = bisect.bisect_left(cam_ts, lidar_ts[i])

        candidates = []
        if pos > 0:
            candidates.append(target_times[pos - 1])
        if pos < len(target_times):
            candidates.append(target_times[pos])

        cam_cand = min(candidates, key=lambda x: abs(x[0] - lidar_ts[i]))

        pairs.append((base_times[i], cam_cand))

    return pairs

def write_pairs(pairs, out_file, b_topic, t_topic):

    dir_path = os.path.dirname(out_file)
    if dir_path != "":
        os.makedirs(dir_path, exist_ok=True)

    # Guardar CSV directamente en la ruta proporcionada
    with open(out_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            f"{b_topic}_timestamp",
            f"{b_topic}_index",
            f"{t_topic}_timestamp",
            f"{t_topic}_index"
        ])

        for (t_b, idx_b), (t_t, idx_t) in pairs:
            writer.writerow([t_b, idx_b, t_t, idx_t])

    print(f"Pairs exported to {os.path.abspath(out_file)}")

def main():
    parser = argparse.ArgumentParser(description="Get images from a rosbag")
    parser.add_argument("--bag_path", type=str, required=True, help="Rosbag path")
    parser.add_argument("--base_topic", type=str, required=True, help="Slowest topic (used as reference to sync the other one)")
    parser.add_argument("--target_topic", type=str, required=True, help="Fastest topic that has to be synchronized with the other one")
    parser.add_argument("--output", type=str, required=True, help="Output path")
    args = parser.parse_args()

    pairs = sync_base_target(
        args.bag_path, args.base_topic, args.target_topic)

    write_pairs(pairs, args.output, args.base_topic, args.target_topic)

if __name__ == "__main__":
    main()
