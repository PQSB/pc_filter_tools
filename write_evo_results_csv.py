import argparse
import pandas as pd
import json
import zipfile
import sys
import os

def get_stats_from_zip(file_path):
    """
    Extracts mean, median, std and max from evo_ape zip results file.
    """
    if not os.path.exists(file_path):
        print(f"[ERROR] File: {file_path} not found")
        return None

    stats = None
    try:
        with zipfile.ZipFile(file_path, 'r') as z:
            with z.open('stats.json') as f:
                stats = json.load(f)
    except zipfile.BadZipFile:
        print(f"Invalid evo zip file")

    if stats:
        return {
            'mean': stats.get('mean', None),
            'median': stats.get('median', None),
            'std': stats.get('std', None),
            'max': stats.get('max', None)
        }
    return None

def main():
    parser = argparse.ArgumentParser(description="Update csv rows with evo_ape results")
    parser.add_argument("--csv", required=True, help="Path to the csv file to update")

    # Needed arguments
    parser.add_argument("--sequence", required=True, help="Valor para la columna sequence.")
    parser.add_argument("--test", type=int, required=True, help="Número de test.")
    parser.add_argument("--experiment", type=int, required=True, help="Número de experiment.")

    # Detector argument, only present sometimes
    parser.add_argument("--detector", type=str, default="", help="Nombre corto del detector (mood, pvrcnn, sk).")

    # evo_ape zip results file
    parser.add_argument("--zip_trans", required=True, help="Translation evo_ape results file")
    parser.add_argument("--zip_rot", required=True, help="Rotation evo_ape results file")
    
    args = parser.parse_args()

    # Changes the detector provieded to the expected detector name in the csv file
    detectors = {
        "mood": "3D-MOOD",
        "pvrcnn": "PV-RCNN",
        "sk": "SEM KITTI"
    }

    # Security processing of the detector name, if provided
    arg_detector = args.detector.strip()
    real_detector = ""

    if arg_detector != "":
        # Check if the detector provided is in the dectors dictionary
        if arg_detector in detectors:
            real_detector = detectors[arg_detector]
        else:
            print(f"[ERROR]: Detector '{arg_detector}' is not valid")
            sys.exit(1)

    info_detector = f"Detector: {real_detector if real_detector else 'NO'}"

    # Get the stats from evo_ape zip results files
    trans_stats = get_stats_from_zip(args.zip_trans)
    rot_stats = get_stats_from_zip(args.zip_rot)

    if trans_stats is None:
        print("[ERROR]: Stats coudn't be loaded from the translation zip file provided")
        sys.exit(1)
    elif rot_stats is None:
        print("[ERROR]: Stats coudn't be loaded from the rotation zip file provided")
        sys.exit(1)

    # Load csv file (making sure Detector and sequence columns are loaded as str)
    try:
        df = pd.read_csv(args.csv, sep=';', decimal=',', encoding='utf-8-sig', dtype={'sequence': str, 'Detector': str})
    except FileNotFoundError:
        print(f"[ERROR]: CSV file '{args.csv}' doesn't exists")
        sys.exit(1)

    # Generate the mask for searching the row in which to write evo_ape results
    mask = (
        (df['sequence'].astype(str) == str(args.sequence)) &
        (df['test'] == args.test) &
        (df['experiment'] == args.experiment)
    )

    # If the detector argument is provided, add it to the mask
    if real_detector != "":
        mask = mask & (df['Detector'] == real_detector)

    # Search for the row that matches the mask
    if not mask.any():
        print(f"[WARN]: NO matching row for: sequence={args.sequence}, test={args.test}, experiment={args.experiment}. ({info_detector})")
        sys.exit(0)

    # Fill the row results columns
    df.loc[mask, 't_mean'] = trans_stats['mean']
    df.loc[mask, 't_median'] = trans_stats['median']
    df.loc[mask, 't_std'] = trans_stats['std']
    df.loc[mask, 't_max'] = trans_stats['max']
    
    df.loc[mask, 'r_mean'] = rot_stats['mean']
    df.loc[mask, 'r_median'] = rot_stats['median']
    df.loc[mask, 'r_std'] = rot_stats['std']
    df.loc[mask, 'r_max'] = rot_stats['max']

    # Save results
    df.to_csv(args.csv, sep=';',  decimal=',', encoding='utf-8', index=False)
    print(f"-> CSV file updated succesfully [Seq: {args.sequence} | Test: {args.test} | Exp: {args.experiment} | {info_detector}]")

if __name__ == "__main__":
    main()
