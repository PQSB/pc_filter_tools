import numpy as np
import argparse
import sys
from utils import load_kitti_matrix

def read_kitti_poses(poses_path):
    """
    Reads txt file with all the poses and returns an array
    with the poses inside a matrices
    """
    poses = []
    try:
        with open(poses_path, 'r') as f:
            for linea in f:
                pose = np.array([float(v) for v in linea.strip().split(' ') if v != ''])
                if len(pose) == 12:
                    pose_matrix = np.eye(4)
                    pose_matrix[:3, :] = pose.reshape(3, 4)
                    poses.append(pose_matrix)
        return poses
    except Exception as e:
        print(f"[ERROR] Failed while processing '{poses_path}': {e}")

    return None

def write_kitti_poses(poses, out_path):
    """
    Store the pose matrices in kitti format (12 values per line)
    """
    with open(out_path, 'w') as f:
        for pose in poses:
            # Just keep the 3x4 part
            datos = pose[:3, :].flatten()
            linea = ' '.join([f"{x:.6e}" for x in datos])
            f.write(linea + '\n')

def main():
    parser = argparse.ArgumentParser(description="Change the reference system of the SLAM poses to align them with kitti odometry ground truth")
    parser.add_argument("--input_file", type=str, required=True, help="Path to the input poses file")
    parser.add_argument("--calib_file", type=str, required=True, help="Path to file with the velo2cam matrix")
    parser.add_argument("--out_file", type=str, required=True, help="Path in which to store the file with aligned poses")
 
    args = parser.parse_args()

    poses = read_kitti_poses(args.input_file)
    if poses is None:
        sys.exit(1)

    Tr_raw = load_kitti_matrix(args.calib_file, "Tr:")
    if Tr_raw is None:
        sys.exit(1)

    Tr = np.eye(4) # Crea una matriz identidad de 4x4 (la traslación por defecto ya es 0)
    Tr[:3, :3] = Tr_raw[:3, :3] # Copia SOLAMENTE la rotación (3x3), ignorando la columna de traslación
    # -------------------------
    Tr_inv = np.linalg.inv(Tr)
    aligned_poses = []

    for pose in poses:
        aligned_pose = Tr @ pose @ Tr_inv
        aligned_poses.append(aligned_pose)

    write_kitti_poses(aligned_poses, args.out_file)
    print(f"[OK] Aligned poses stored in: {args.out_file}")

if __name__ == "__main__":
    main()
