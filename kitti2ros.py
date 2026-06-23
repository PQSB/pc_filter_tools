import argparse
import numpy as np
import sys
import os

def convert_kitti_to_xy_plane(input_file, output_file):
    # 1. Define the transformation matrix (KITTI to ROS/Standard XY plane)
    # Maps: X->-Y, Y->-Z, Z->X
    T_trans = np.array([
        [ 0,  0,  1,  0],
        [-1,  0,  0,  0],
        [ 0, -1,  0,  0],
        [ 0,  0,  0,  1]
    ], dtype=float)
    
    # The inverse is just the transpose of the rotation part
    T_trans_inv = np.linalg.inv(T_trans)

    converted_poses = []

    # 2. Read the original KITTI data
    if not os.path.exists(input_file):
        print(f"Error: The input file '{input_file}' was not found.")
        sys.exit(1)

    print(f"Reading from: {input_file} ...")
    with open(input_file, 'r') as f:
        for line in f:
            # Parse the 12 numbers into a flat array
            vals = np.array([float(x) for x in line.strip().split()])
            
            if len(vals) != 12:
                continue # Skip empty or invalid lines
                
            # Reshape into 3x4 and convert to 4x4 homogenous matrix
            T_pose = np.eye(4)
            T_pose[:3, :4] = vals.reshape(3, 4)
            
            # 3. Apply the transformation
            # T_left rotates the world map (translation)
            # T_right (inverse) rotates the sensor's local frame (orientation)
            T_new = T_trans @ T_pose @ T_trans_inv
            
            # Extract the new 3x4 matrix and flatten it back to 12 elements
            new_vals = T_new[:3, :4].flatten()
            
            # Format as space-separated string using scientific notation
            line_str = " ".join([f"{v:.6e}" for v in new_vals])
            converted_poses.append(line_str)

    # 4. Save the converted data
    print(f"Saving to: {output_file} ...")
    with open(output_file, 'w') as f:
        for pose in converted_poses:
            f.write(pose + "\n")
            
    print(f"Successfully converted {len(converted_poses)} poses!")

def main():
    # Set up argparse for command-line arguments
    parser = argparse.ArgumentParser(
        description="Convert a KITTI odometry ground truth file from the XZ plane (camera frame) to the standard XY plane (ROS frame)."
    )
    
    # Add positional arguments for input and output files
    parser.add_argument(
        "input_file", 
        type=str, 
        help="Path to the original KITTI odometry .txt file (e.g., 00.txt)"
    )
    
    parser.add_argument(
        "output_file", 
        type=str, 
        help="Path where the converted .txt file will be saved (e.g., 00_ros.txt)"
    )

    # Parse the arguments from the command line
    args = parser.parse_args()

    # Run the conversion
    convert_kitti_to_xy_plane(args.input_file, args.output_file)

if __name__ == "__main__":
    main()

