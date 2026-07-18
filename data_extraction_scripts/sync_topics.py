import argparse
import bisect
import os
import csv
from tqdm import tqdm
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def store_topics_times(path, b_topic, t_topics):
    # Configure reader
    if os.path.isdir(path):
        files = os.listdir(path)
        if any(f.endswith('.mcap') for f in files):
            storage_id = 'mcap'
        else:
            storage_id = 'sqlite3'
    else:
        sys.exit(f"Error: Invalid path, not a directory: {path}")

    storage_options = rosbag2_py.StorageOptions(
        uri=path,
        storage_id=storage_id
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Check that the topics exist in the rosbag
    if b_topic not in type_map:
        print(f"❌ Topic: {b_topic} doesn't exist")
        sys.exit(1)

    for t in t_topics:
        if t not in type_map:
            print(f"❌ Topic: {t} doesn't exist")
            sys.exit(1)

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

    # Iterate through the rosbag, storing the messages from each topic in an array
    while reader.has_next():
        topic, data, t = reader.read_next()
        pbar.update(1)

        if topic == b_topic:
            msg = deserialize_message(data, base_msg_type)
            time = (int(msg.header.stamp.sec) * 1_000_000_000) + int(msg.header.stamp.nanosec)

            base_msgs.append((time, base_idx))
            base_idx += 1

        elif topic in t_topics:
            msg = deserialize_message(data, target_msg_type[topic])
            time = (int(msg.header.stamp.sec) * 1_000_000_000) + int(msg.header.stamp.nanosec)

            target_msgs[topic].append((time, targets_idx[topic]))
            targets_idx[topic] += 1

    return base_msgs, target_msgs


def sync_base_targets(path, b_topic, t_topics, times_file, gen_csv):
    base_times, target_times = store_topics_times(path, b_topic, t_topics)

    if not base_times:
        print(f"❌ Error: Topic '{b_topic}' is empty")
        sys.exit(1)

    if times_file:
        dir_path = os.path.dirname(times_file)
        if dir_path != "":
            os.makedirs(dir_path, exist_ok=True)

        t0 = base_times[0][0]

        # Create an array with the converted timestamps
        lines = [f"{((time_ns - t0) / 1_000_000_000.0):.9f}" for time_ns, _ in base_times]

        # Join all the timestamps using \n as separator
        with open(times_file, "w") as f:
            f.write("\n".join(lines))

        print(f"✅ timestamps file exported to {os.path.abspath(times_file)}")

    if not gen_csv:
        return None

    # Extract only the timestamps of every target topic
    targets_ts = {}
    for topic in t_topics:
        ts_idx = target_times[topic]
        ts_only = [ts for (ts, _) in ts_idx]
        targets_ts[topic] = ts_only

    for topic in t_topics:
        if not targets_ts[topic]:
            print(f"❌ Error: Topic {topic} is empty")
            sys.exit(1)
    
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
            if pos < len(ts_and_idx):
                candidates.append(ts_and_idx[pos])

            best = min(candidates, key=lambda x: abs(x[0] - ts_base))

            row[t] = best

        pairs.append(row)

    return pairs

def write_pairs(pairs, out_file, b_topic, t_topics):
    if pairs is None:
        return

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
                line.append(str(ts_topic))
                line.append(str(idx_topic))

            writer.writerow(line)

    print(f"Pairs exported to {os.path.abspath(out_file)}")

def main():
    parser = argparse.ArgumentParser(description="Sync a list of target topics with a base topic and export a KITTI format base topic times file")
    parser.add_argument("--bag_path", type=str, required=True, help="Rosbag path")
    parser.add_argument("--base_topic", type=str, required=True, help="Slowest topic (used as reference to sync the other one)")
    parser.add_argument("--target_topics", type=str, nargs="+", help="Topics to be synchronized with the base one (/topic1 /topic2 ...)")
    parser.add_argument("--out_times", type=str, help="Output path for times file of the reference topic")
    parser.add_argument("--out_sync_file", type=str, help="Output path of the csv file")
    args = parser.parse_args()

    if not args.out_times and not args.out_sync_file:
        parser.error("Must provide at least one of the output arguments: --out_times or --out_sync_file")

    if args.out_sync_file and not args.target_topics:
        parser.error("If --out_sync_file is active, --target_topics must be provided")

    gen_csv = args.out_sync_file is not None

    pairs = sync_base_targets(
        args.bag_path, args.base_topic, args.target_topics, args.out_times, gen_csv)

    write_pairs(pairs, args.out_sync_file, args.base_topic, args.target_topics)

if __name__ == "__main__":
    main()
