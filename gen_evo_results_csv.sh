#!/bin/bash

set -e

if [ "$#" -ne 7 ]; then
    echo "Usage: $0 <ROOT_POSES_DIR> <GT_DIR> <CONDA_ENV> <OUTPUT_ROOT> <SEQ_START> <SEQ_END> <CSV_FILE>"
    exit 1
fi

ROOT_POSES_DIR="$1"
GT_DIR="$2"
CONDA_ENV="$3"
OUTPUT_ROOT="$4"

ORIGINAL_CSV_FILE="$7"

# Get the sequence in decimal format
SEQ_START=$((10#$5))
SEQ_END=$((10#$6))

CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh" 

if [ -n "$CONDA_DEFAULT_ENV" ] && [ "$CONDA_DEFAULT_ENV" == "$CONDA_ENV" ]; then
    echo -e "\e[33m$CONDA_ENV conda environment already active\e[0m"
else
    conda activate "$CONDA_ENV"
    echo -e "\e[34m$CONDA_ENV conda environment\e[0m \e[32mcorrectly activated\e[0m"
fi

if [ ! -f "$ORIGINAL_CSV_FILE" ]; then
    echo "[ERROR]: The csv file provided doesn't exists: $ORIGINAL_CSV" >&2
    exit 1
fi

# Make a copy instead of filling the provided csv file
CSV_FILE="${ORIGINAL_CSV_FILE%.csv}_filled.csv"
cp "$ORIGINAL_CSV_FILE" "$CSV_FILE"

# Iterate through the subdirectories inside the root poses dir
for subdir in "$ROOT_POSES_DIR"/*/; do
    [ -d "$subdir" ] || continue

    # Store subdirectory name (seq_{nseq})
    nseq_dir=$(basename "$subdir")

    # Get the sequence number
    if [[ "$nseq_dir" =~ ([0-9]+) ]]; then
        raw_nseq="${BASH_REMATCH[1]}" # Stores "00", "01", "10", etc.
    else
        echo "Ignoring $nseq_dir due to invalid name"
        continue
    fi

    # Get the clean sequence number (ex: 0 from 00)
    nseq_clean=$((10#$raw_nseq))

    # Check if the sequence is in the range provided
    if [ "$nseq_clean" -lt "$SEQ_START" ] || [ "$nseq_clean" -gt "$SEQ_END" ]; then
        continue
    fi

    # Build the path to the current sequence ground truth poses file
    gt_file="$GT_DIR/seq_${raw_nseq}_ground_truth_poses.txt"

    if [ ! -f "$gt_file" ]; then
        echo "ERROR: ground truth file: $gt_file not found" >&2
        exit 1
    fi

    echo "========================================="
    echo "Processing Sequence ${raw_nseq} results:"
    echo "Using ground truth file: ${gt_file}"
    echo "========================================="

    # Create output subdirectory for current sequence
    out_subdir="$OUTPUT_ROOT/seq_${raw_nseq}"
    mkdir -p "$out_subdir"

    # Iterate through the files inside the current sequence subdirectory
    for filepath in "$subdir"/*; do
        [ -f "$filepath" ] || continue
        filename=$(basename "$filepath")

        # Check whether the file name has the correct structure
        if [[ "$filename" =~ ^seq([0-9]+)_test([0-9]+)_exp([0-9]+)_?([a-zA-Z0-9]*?)_?poses_kitti\.[a-zA-Z0-9]+$ ]]; then
            # Get the data from the filename
            raw_seq="${BASH_REMATCH[1]}"
            raw_test="${BASH_REMATCH[2]}"
            raw_exp="${BASH_REMATCH[3]}"
            optional="${BASH_REMATCH[4]}"

            # Create the base name for current sequence evo_ape results files
            if [ -z "$optional" ]; then
                base_name="seq${raw_seq}_test${raw_test}_exp${raw_exp}_results"
            else
                base_name="seq${raw_seq}_test${raw_test}_exp${raw_exp}_${optional}_results"
            fi

            ape_out_zip_trans="$out_subdir/${base_name}_ape_trans.zip"
            ape_out_zip_rot="$out_subdir/${base_name}_ape_rot.zip"
            rpe_out_zip_trans="$out_subdir/${base_name}_rpe_trans.zip"
            rpe_out_zip_rot="$out_subdir/${base_name}_rpe_rot.zip"

            # Get evo_ape translation and rotation results
            echo "Processing : $filename"
            evo_ape kitti "$gt_file" "$filepath" -r trans_part --save_results "$ape_out_zip_trans"
            evo_ape kitti "$gt_file" "$filepath" -r rot_part --save_results "$ape_out_zip_rot"

            # Get evo_rpe translation and rotation results
            evo_rpe kitti "$gt_file" "$filepath" -r trans_part --save_results "$rpe_out_zip_trans"
            evo_rpe kitti "$gt_file" "$filepath" -r rot_part --save_results "$rpe_out_zip_rot"

            clean_seq=$((10#$raw_seq))
            clean_test=$((10#$raw_test))
            clean_exp=$((10#$raw_exp))

            python write_evo_results_csv.py \
                --csv "$CSV_FILE" \
                --sequence "$clean_seq" \
                --test "$clean_test" \
                --experiment "$clean_exp" \
                --detector "$optional" \
                --zip_ape_trans "$ape_out_zip_trans" \
                --zip_ape_rot "$ape_out_zip_rot" \
                --zip_rpe_trans "$rpe_out_zip_trans" \
                --zip_rpe_rot "$rpe_out_zip_rot"

        else
            echo "Error: Invalid file in directory $filename" >&2
            exit 1
        fi
    done
done

conda deactivate
echo "EVO RESULTS CSV WRITTING COMPLETED!"
