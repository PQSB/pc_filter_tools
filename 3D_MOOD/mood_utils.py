import numpy as np
import yaml

def load_intrinsics_from_yaml(path):
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        K = np.array(data["camera"]["K"], dtype=np.float32).reshape(3, 3)
        return K

    except Exception as e:
        print(f"[ERROR] Failed to load matrix K '{path}'. Details: {e}")

    return None

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
