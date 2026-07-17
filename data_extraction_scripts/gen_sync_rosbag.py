import csv
import rosbag2_py
import argparse
import os
import sys

from tqdm import tqdm

# def clean(topic):
#     return topic.replace("/", "_").strip("_")

def detect_topics_from_csv(csv_path):
    """
    Returns a list with the topics to sync
    """
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames

    topics = []
    for name in fieldnames:
        if name.endswith("_index"):
            topic = name.replace("_index", "")
            topics.append(topic)

    return topics

def load_indices(csv_path):
    """
    Returns a dict:
    {
        "topic_clean_name": {0, 5, 7, ...},
        ...
    }
    """
    indices = {}

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in row:
                if key.endswith("_index"):
                    topic = key.replace("_index", "")
                    if topic not in indices:
                        indices[topic] = set()
                    indices[topic].add(int(row[key]))

    return indices

def generate_synced_bag(input_bag, output_bag, csv_path):
    # -----------------------------
    # 1. Detect topics to sync
    # -----------------------------
    topics_to_filter = detect_topics_from_csv(csv_path)
    print("Topics to sync:", topics_to_filter)

    indices = load_indices(csv_path)

    # Counters for each topic to filter
    counters = {topic: 0 for topic in topics_to_filter}

    if os.path.isdir(input_bag):
        files = os.listdir(input_bag)
        if any(f.endswith('.mcap') for f in files):
            storage_id = 'mcap'
        else:
            storage_id = 'sqlite3'
    else:
        sys.exit(f"Error: Invalid path, not a directory: {input_bag}")
    # -----------------------------
    # 2. Configure reader
    # -----------------------------
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # -----------------------------
    # 3. Configure writer
    # -----------------------------
    writer = rosbag2_py.SequentialWriter()

    out_storage = rosbag2_py.StorageOptions(uri=output_bag, storage_id=storage_id)
    out_converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    writer.open(out_storage, out_converter)

    # Register all topics
    for t in topic_types:
        writer.create_topic(rosbag2_py.TopicMetadata(
            0,
            t.name,
            t.type,
            "cdr",
            t.offered_qos_profiles
        ))

    # -----------------------------
    # 4. Read the rosbag and filter it
    # -----------------------------

    pbar = tqdm(
        desc="Generating filtered rosbag",
        dynamic_ncols=True,
        total=None,
        mininterval=0.2,
        smoothing=0.3,
        unit="msg"
    )

    while reader.has_next():
        topic, data, t = reader.read_next()
        pbar.update(1)

        # clean_name = clean(topic)

        # If the topic is in the CSV file filter by index
        if topic in topics_to_filter:
            idx = counters[topic]
            if idx in indices[topic]:
                writer.write(topic, data, t)
            counters[topic] += 1

        else:
            # Copy everything of the other topics
            writer.write(topic, data, t)

def main():
    parser = argparse.ArgumentParser(description="Generate a new rosbag filtering messages")
    parser.add_argument("--input_bag", type=str, required=True, help="Rosbag path")
    parser.add_argument("--sync_file", type=str, required=True, help="File with the topics messages relation")
    parser.add_argument("--out_bag", type=str, required=True, help="Output path")
    args = parser.parse_args()

    generate_synced_bag(args.input_bag, args.out_bag, args.sync_file)

    print("Finished generating filtered rosbag.")

if __name__ == "__main__":
    main()
