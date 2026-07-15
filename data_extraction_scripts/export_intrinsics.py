import argparse
import yaml
import numpy as np
import rosbag2_py
from sensor_msgs.msg import CameraInfo
from rclpy.serialization import deserialize_message

def extract_K_from_camera_info(msg_bytes):
    """Deserialize CameraInfo and extract K."""
    msg = deserialize_message(msg_bytes, CameraInfo)
    K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
    return K

def export_intrinsics(bag_path, topic, output_yaml):
    # Configure reader
    if bag_path.endswith('.mcap'):
        storage_id = 'mcap'
    else:
        storage_id = 'sqlite3'

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Read messages until the desired topic is reached
    K = None
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t == topic:
            K = extract_K_from_camera_info(data)
            break

    if K is None:
        raise RuntimeError(f"Topic'{topic}' not found in rosbag.")

    # Store YAML
    intrinsics_dict = {
        "camera": {
            "K": K.flatten().tolist()
        }
    }

    with open(output_yaml, "w") as f:
        yaml.dump(intrinsics_dict, f)

    print(f"K matrix exported to {output_yaml}")
    print("K =\n", K)

def main():
    parser = argparse.ArgumentParser(description="Export K matrix from a rosbag")
    parser.add_argument("--bag", type=str, required=True, help="Rosbag path")
    parser.add_argument("--topic", type=str, required=True, help="Topic from which to obtain the matrix")
    parser.add_argument("--output", type=str, required=True, help="Output .yaml file")

    args = parser.parse_args()

    # Check the file extension is correct
    if not args.output.endswith(".yaml"):
        raise ValueError("Output file must have .yaml extension")

    export_intrinsics(args.bag, args.topic, args.output)

if __name__ == "__main__":
    main()
