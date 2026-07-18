import os
import cv2
import argparse

def preprocess_data(calib_path, image_path, out_path, p_prefix, tr_prefix):
    """
    Reads an image and a kitti odometry calibration file, generating an optimized file
    for filter_cloud_by_detections_from_dataset filter_fov option (width, height, 12 P values, 12 Tr values).
    """

    print(f"Processing\n - Image: {image_path}\n - Calib: {calib_path}\n - p_prefix: {p_prefix}\n - tr_prefix: {tr_prefix}")

    img = cv2.imread(image_path)

    if img is None:
        print(f"[ERROR] Can't read image: '{image_path}'")
        return

    h, w, _ = img.shape

    # Get P and Tr values from the kitti calib file
    p_data = []
    tr_data = []

    try:
        with open(calib_path, 'r') as f:
            for line in f:
                # Remove blank sapces
                clean_line = line.strip()

                if not clean_line:
                    continue
                
                if clean_line.startswith(p_prefix):
                    p_data = clean_line.split()[1:]
                elif clean_line.startswith(tr_prefix):
                    tr_data = clean_line.split()[1:]

    except FileNotFoundError:
        print(f"[ERROR] The calibration file was not found in: '{calib_path}'")
        return

    # Security check
    if len(p_data) != 12:
        print(f"[ERROR] Missing data in {p_prefix}. Found {len(p_data)} out of 12.")
        return
    if len(tr_data) != 12:
        print(f"[ERROR] Missing data in {tr_prefix}. Found {len(tr_data)} out of 12.")
        return

    # Order: [width, height, P_1...12, Tr_1...12]
    final_data = [w, h] + p_data + tr_data
    
    # Create output directory if it doesn't exists
    out_dir = os.path.dirname(out_path)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    try:
        with open(out_path, 'w') as f:
            # Turn numbers into text separated by one blank space
            f.write(" ".join(map(str, final_data)))
            
        print(f"[OK] Preprocessed data file stored in: '{out_path}'")
        print(f"  -> Image dimensions: {w}x{h} px")
        
    except Exception as e:
        print(f"[ERROR] The destination file could not be saved: {e}")

def main():
    parser = argparse.ArgumentParser(description="Generate an optimized file for filter_cloud_by_detections_from_dataset filter_fov option")

    parser.add_argument("--calib_file", type=str, required=True, help="kitti format calib file path")
    parser.add_argument("--image", type=str, required=True, help="path to the image")
    parser.add_argument("--out_path", type=str, required=True, help="path to store the resulting preprocessed file")
    parser.add_argument("--tr_prefix", type=str, default="Tr:", help="Prefix of the velo2cam matrix in the calib file")
    parser.add_argument("--p_prefix", type=str, default="P2:", help="Prefix of the camera matrix in the calib file")

    args = parser.parse_args()

    preprocess_data(args.calib_file, args.image, args.out_path, args.p_prefix, args.tr_prefix)

if __name__ == "__main__":
    main()
