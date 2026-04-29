import argparse
import bisect
import os
import csv
from tqdm import tqdm

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def store_topics_times(path, b_topic, t_topics):
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

    for t in t_topics:
        if t not in type_map:
            print(f"❌ Topic: {t} doesn't exist")
            return

    target_msgs = {t: [] for t in t_topics}
    base_msgs = []

    target_msg_type = {t: get_message(type_map[t]) for t in t_topics}
    base_msg_type = get_message(type_map[b_topic])

    targets_idx = {t: 0 for t in t_topics}
    base_idx = 0

    pbar = tqdm(
        desc="Reading rosbag",
        dynamic_ncols=True,
        total=None,
        mininterval=0.2,
        smoothing=0.3,
        unit="msg",
        colour="cyan"
    )

    # Recorrer el rosbag guardando en un array los mensajes de cada topic
    while reader.has_next():
        topic, data, t = reader.read_next()
        pbar.update(1)

        if topic == b_topic:
            msg = deserialize_message(data, base_msg_type)
            time = (msg.header.stamp.sec * 1_000_000_000) + msg.header.stamp.nanosec
            
            base_msgs.append((time, base_idx))
            base_idx += 1

        elif topic in t_topics:
            msg = deserialize_message(data, target_msg_type[topic])
            time = (msg.header.stamp.sec * 1_000_000_000) + msg.header.stamp.nanosec

            target_msgs[topic].append((time, targets_idx[topic]))
            targets_idx[topic] += 1

    return base_msgs, target_msgs


def sync_base_targets(path, b_topic, t_topics):
    base_times, target_times = store_topics_times(path, b_topic, t_topics)

    # Extract only the timestamps of every target topic
    targets_ts = {}
    for topic in t_topics:
        ts_idx = target_times[topic]
        ts_only = [ts for (ts, _) in ts_idx]
        targets_ts[topic] = ts_only

    pairs = []

    for i in tqdm(range(len(base_times)), desc="Synchronizing topics"):
        ts_base, idx_base = base_times[i]

        row = {"base": (ts_base, idx_base)}

        for t in t_topics:
            ts_and_idx = target_times[t]

            pos = bisect.bisect_left(targets_ts[t], ts_base)

            candidates = []
            if pos > 0:
                candidates.append(ts_and_idx[pos - 1])
            if pos < len(target_times):
                candidates.append(ts_and_idx[pos])

            best = min(candidates, key=lambda x: abs(x[0] - ts_base))

            row[t] = best

        pairs.append(row)

    return pairs

def write_pairs(pairs, out_file, b_topic, t_topics):

    dir_path = os.path.dirname(out_file)
    if dir_path != "":
        os.makedirs(dir_path, exist_ok=True)

    # Store the CSV in the provided path
    with open(out_file, "w", newline="") as f:
        writer = csv.writer(f)

        # Write the headers in the file
        header = [
            f"{b_topic}_timestamp",
            f"{b_topic}_index"
        ]

        for topic in t_topics:
            header.append(f"{topic}_timestamp")
            header.append(f"{topic}_index")

        writer.writerow(header)

        # Write every row
        for row in pairs:
            ts_base, idx_base = row["base"]
            line = [ts_base, idx_base]

            for t in t_topics:
                ts_topic, idx_topic = row[t]
                line.append(ts_topic)
                line.append(idx_topic)

            writer.writerow(line)

    print(f"Pairs exported to {os.path.abspath(out_file)}")

def main():
    parser = argparse.ArgumentParser(description="Sync a list of target topics with a base topic")
    parser.add_argument("--bag_path", type=str, required=True, help="Rosbag path")
    parser.add_argument("--base_topic", type=str, required=True, help="Slowest topic (used as reference to sync the other one)")
    parser.add_argument("--target_topics", type=str, required=True, nargs="+", help="Topics to be synchronized with the base one (/topic1 /topic2 ...)")
    parser.add_argument("--output", type=str, required=True, help="Output path of the csv file")
    args = parser.parse_args()

    pairs = sync_base_targets(
        args.bag_path, args.base_topic, args.target_topics)

    write_pairs(pairs, args.output, args.base_topic, args.target_topics)

if __name__ == "__main__":
    main()
