import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs_py import point_cloud2
import warnings

warnings.simplefilter("once", UserWarning)

MAX_INTENSITY = 1.0

bridge = CvBridge()

def depth_2_cloud(depth_img, K, scale=False, organized=False):
    if K is None:
        return

    if depth_img.dtype == np.uint16:
        # Change to meters 16UC1 → milimiters
        depth = depth_img.astype(np.float32) / 1000.0
    else:
        depth = depth_img.astype(np.float32)

    H, W = depth.shape

    # magic_num = 3
    # magic_num = 640 / 1920
    if scale:
        scale_x = 1920/640
        scale_y = 1080/480
    else:
        scale_x = 1
        scale_y = 1

    # Escala de la imagen de color con la imagen derecha 1920/640
    # fx, fy = K[0, 0]*scale_x, K[1, 1]*scale_x
    # cx, cy = K[0, 2]*scale_y, K[1, 2]*scale_y

    fx, fy = K[0, 0]*scale_x, K[1, 1]*scale_y
    cx, cy = K[0, 2]*scale_x, K[1, 2]*scale_y

    u, v = np.meshgrid(np.arange(W), np.arange(H))

    Z = depth
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    # Change coordinates to LIDAR system
    xlidar = Z
    ylidar = -X
    zlidar = -Y

    if organized:
        # Oragnized structure (H, W, 4)
        intensity = np.full_like(xlidar, MAX_INTENSITY, dtype=np.float32)
        cloud = np.stack((xlidar, ylidar, zlidar), axis=-1)
    else:
        # Unorganized structure (N, 4) filtering invalid values
        mask = (xlidar > 0) & (~np.isnan(xlidar))
        intensity = np.full(np.count_nonzero(mask), MAX_INTENSITY, dtype=np.float32)

        cloud = np.column_stack((xlidar[mask], ylidar[mask], zlidar[mask], intensity))

    return cloud

def pointcloud2_to_xyzi(msg):
    pc_fields = [f.name for f in msg.fields]

    # Check if the input point cloud has intensity field
    if "intensity" in pc_fields:
        points = list(point_cloud2.read_points_numpy(
        msg,
        field_names=("x", "y", "z", "intensity"),
        skip_nans=True
        ))

    # When the point cloud has no intensity field
    else:
        warnings.warn(
            "Input point cloud has no 'intensity' field. "
            "A default intensity value (255) will be assigned to all points. "
            "This value is synthetic and should not be interpreted as a real sensor measurement.",
            UserWarning,
            stacklevel=2
        )

        xyz = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        # Generate generic intensity field
        intensity = np.full((xyz.shape[0], 1), MAX_INTENSITY, dtype=np.float32)
        points = np.hstack((xyz, intensity))

    return np.array(points, dtype=np.float32)

def export_cloud(msg, out_dir):
    ts = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    filename = os.path.join(out_dir, f"{ts}.bin")

    cloud = pointcloud2_to_xyzi(msg)
    cloud.astype(np.float32).tofile(filename)

    return

def export_cloud_from_depth(rosdepth, k, outdir, organized=False):
    ts = rosdepth.header.stamp.sec * 1_000_000_000 + rosdepth.header.stamp.nanosec
    filename = os.path.join(outdir, f"{ts}.bin")

    depth_img = bridge.imgmsg_to_cv2(rosdepth, desired_encoding="passthrough")

    depth_cloud = depth_2_cloud(depth_img, k, True, organized)
    depth_cloud.astype(np.float32).tofile(filename)

    return

def export_image(ros_msg, out_dir):
    cv_image = bridge.imgmsg_to_cv2(ros_msg, desired_encoding='bgr8')

    ts = ros_msg.header.stamp.sec * 1_000_000_000 + ros_msg.header.stamp.nanosec

    filename = os.path.join(out_dir, f"{ts}.png")
    cv2.imwrite(filename, cv_image)

    return

def export_depth_image(msg, out_dir):
    depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    if depth_img.dtype == np.uint16:
        depth_in_mm = depth_img

    else:
        depth_in_mm = depth_img.astype(np.float32) * 1000.0

        depth_in_mm = np.nan_to_num(
            depth_in_mm,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        depth_in_mm = np.clip(depth_in_mm, 0, 65535)
        depth_in_mm = depth_in_mm.astype(np.uint16)

    ts = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    out_img_path = os.path.join(out_dir, f"{ts}.png")

    cv2.imwrite(out_img_path, depth_in_mm)
