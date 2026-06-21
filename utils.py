import numpy as np
import yaml
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs_py import point_cloud2

bridge = CvBridge()

def load_intrinsics_from_yaml(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    K = np.array(data["camera"]["K"], dtype=np.float32).reshape(3, 3)
    return K

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
        # Oragnized structure (H, W, 3)
        cloud = np.stack((xlidar, ylidar, zlidar), axis=-1)
    else:
        # Unorganized structure (N, 3) filtering invalid values
        mask = (xlidar > 0) & (~np.isnan(xlidar))
        cloud = np.column_stack((xlidar[mask], ylidar[mask], zlidar[mask]))

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
    #bridge = CvBridge()

    ts = rosdepth.header.stamp.sec * 1_000_000_000 + rosdepth.header.stamp.nanosec
    filename = os.path.join(outdir, f"{ts}.bin")

    depth_img = bridge.imgmsg_to_cv2(rosdepth, desired_encoding="passthrough")

    depth_cloud = depth_2_cloud(depth_img, k, True, organized)
    depth_cloud.astype(np.float32).tofile(filename)

    return

def export_image(ros_msg, out_dir):
    #bridge = CvBridge()
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

def load_kitti_matrix(calib_file, prefix):
    if not prefix.endswith(':'):
        prefix += ':'

    try:
        with open(calib_file, "r") as f:
            for line in f:
                line = line.strip()

                if line.startswith(prefix):
                    return np.array([float(v) for v in line.split()[1:]]).reshape(3, 4).astype(np.float32)

    except Exception as e:
        print(f"[ERROR] Failed to load '{prefix}' from '{calib_file}'. Details: {e}")

    return None

def get_cam2_2_lidar_matrix(P2, T_cam0_to_lidar):
    fx = P2[0, 0]
    fy = P2[1, 1]

    bx = P2[0, 3] / fx
    by = P2[1, 3] / fy
    bz = P2[2, 3]

    t2 = np.array([bx, by, bz])

    # construir matriz Cam0 -> Cam2
    T_cam0_to_cam2 = np.eye(4) # no hay rotación de cam0 a cam2
    T_cam0_to_cam2[:3, 3] = t2

    # construir matriz Cam2 -> Cam0
    T_cam2_to_cam0 = np.linalg.inv(T_cam0_to_cam2)

    # Construir matriz global Cam2 -> LIDAR
    T_cam2_to_lidar = T_cam0_to_lidar @ T_cam2_to_cam0

    return T_cam2_to_lidar
