import numpy as np
import yaml
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs_py import point_cloud2

def load_intrinsics_from_yaml(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    K = np.array(data["camera"]["K"], dtype=np.float32).reshape(3, 3)
    return K

def depth_2_cloud(depth_img, K, organized=False):
    if K is None:
        return

    if depth_img.dtype == np.uint16:
        # Change to meters 16UC1 → milimiters
        depth = depth_img.astype(np.float32) / 1000.0
    else:
        depth = depth_img.astype(np.float32)

    H, W = depth.shape

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    u, v = np.meshgrid(np.arange(W), np.arange(H))

    Z = depth
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    if organized:
        # Oragnized structure (H, W, 3)
        cloud = np.stack((X, Y, Z), axis=-1)
    else:
        # Unorganized structure (N, 4) filtering invalid values
        mask = (Z > 0) & (~np.isnan(Z))
        cloud = np.column_stack((X[mask], Y[mask], Z[mask]))

    return cloud

def pointcloud2_to_xyz(msg):
    points = list(point_cloud2.read_points_numpy(
    msg,
    # field_names=("x", "y", "z", "intensity"),
    field_names=("x", "y", "z"),
    skip_nans=True
    ))
    return np.array(points, dtype=np.float32)

def export_cloud(msg, out_dir):
    ts = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    filename = os.path.join(out_dir, f"{ts}.bin")

    cloud = pointcloud2_to_xyz(msg)
    cloud.astype(np.float32).tofile(filename)

    return

def export_cloud_from_depth(rosdepth, k, outdir, organized=False):
    bridge = CvBridge()

    ts = rosdepth.header.stamp.sec * 1_000_000_000 + rosdepth.header.stamp.nanosec
    filename = os.path.join(outdir, f"{ts}.bin")

    depth_img = bridge.imgmsg_to_cv2(rosdepth, desired_encoding="passthrough")

    depth_cloud = depth_2_cloud(depth_img, k, organized)
    depth_cloud.astype(np.float32).tofile(filename)

    return

def export_image(ros_msg, out_dir):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(ros_msg, desired_encoding='bgr8')

    ts = ros_msg.header.stamp.sec * 1_000_000_000 + ros_msg.header.stamp.nanosec

    filename = os.path.join(out_dir, f"{ts}.png")
    cv2.imwrite(filename, cv_image)

    return
