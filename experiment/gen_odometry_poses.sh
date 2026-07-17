#!/bin/bash

set -e

# Check arguments
if [ "$#" -lt 6 ]; then
    echo -e "\e[31mError: Missing parameters.\e[0m"
    echo "Usage: $0 <init_seq> <end_seq> <out_root_dir> <conda_genz_env> <dataset_root_path> <data_files_root_path> [test_selector ('1,2', '0 2', all)]"
    exit 1
fi

# Assign arguments
SEQ_START=$1
SEQ_END=$2
BASE_DEST_DIR=$3
CONDA_ENV=$4
DATASET_ROOT=$5
DATA_ROOT=$6

# Default option is all
TEST_SELECTOR=${7:-"all"}

MOOD_MIN_SCORE=0.15
PV_RCNN_MIN_SCORE=0.4
MAX_DIST=20

# Validate TEST_SELECTOR format
if [[ ! "$TEST_SELECTOR" =~ ^(all|[0-9]+([[:space:],][0-9]+)*)$ ]]; then
    echo -e "\e[31mError: Invalid test_selector format.\e[0m"
    echo "Accepted formats: 'all', digits, or digits separated by commas/spaces (e.g., '1,2', '0 10')."
    exit 1
fi

mkdir -p "$BASE_DEST_DIR"

CONDA_BASE=$(conda info --base 2>/dev/null)

source "$CONDA_BASE/etc/profile.d/conda.sh" 

export genz_icp_out_dir=$BASE_DEST_DIR/tmp_genz

if [ -n "$CONDA_DEFAULT_ENV" ] && [ "$CONDA_DEFAULT_ENV" == "$CONDA_ENV" ]; then
    echo -e "\e[33m$CONDA_ENV conda environment already active\e[0m"
else
    conda activate "$CONDA_ENV"
        echo -e "\e[34m$CONDA_ENV conda environment\e[0m \e[32mcorrectly activated\e[0m"
fi

move_and_clean() {
    local dir_ros2=$1
    cd "${genz_icp_out_dir}/latest"
    mv *_kitti.txt "$SEQ_DEST_DIR/"
    cd - > /dev/null
    rm -rf "${genz_icp_out_dir}"
    rm -r "$dir_ros2"
}

echo "Generating odometry poses from sequence $SEQ_START to $SEQ_END"
echo "Selecting execution mode: TEST(S) $TEST_SELECTOR"
echo "Saving results in: $BASE_DEST_DIR"
echo "--------------------------------------------------"

for i in $(seq "$SEQ_START" "$SEQ_END"); do
    SEQ=$(printf "%02d" $i)

    SEQ_DEST_DIR="$(cd "$BASE_DEST_DIR" && pwd)/seq_${SEQ}"
    mkdir -p "$SEQ_DEST_DIR"

    echo "========================================="
    echo "Processing Sequence: ${SEQ}"
    echo "Saving in: $SEQ_DEST_DIR"
    echo "========================================="

    if echo "$TEST_SELECTOR" | grep -qE '\<(0|all)\>'; then
        # ---------------------- TEST 0 ----------------------------------
        echo "---------------------- TEST 0 ----------------------------------"


        # EXPERIMENT 1 ----------------------------------
        echo "EXPERIMENT 1 ----------------------------------"

        echo "ORIGINAL POINT CLOUD"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --pc_topic /in_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test0_exp1" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test0_exp1/" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test0_exp1"

        #  EXPERIMENT 2 ----------------------------------
        echo "EXPERIMENT 2 ----------------------------------"

        echo "ORIGINAL POINT CLOUD"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --pc_topic /in_clouds --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" -o "${BASE_DEST_DIR}/seq${SEQ}_test0_exp2" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test0_exp2" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test0_exp2"
    fi

    if echo "$TEST_SELECTOR" | grep -qE '\<(1|all)\>'; then
        # ---------------------- TEST 1 ----------------------------------
        echo "---------------------- TEST 1 ----------------------------------"

        # EXPERIMENT 1 ----------------------------------
        echo "EXPERIMENT 1 ----------------------------------"

        # 3D-MOOD
        echo "3D-MOOD"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --m_det_dir "${DATA_ROOT}/3D_MOOD_detections/kitti_odometry/CarPedCyc/seq_${SEQ}/" --m_score $MOOD_MIN_SCORE --m_classes Car,Pedestrian,Cyclist --m_topic /mood_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_mood" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_mood" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_mood"

        # PV-RCNN
        echo "PV-RCNN"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --m_det_dir "${DATA_ROOT}/openPCDet_detections/pv_rcnn/kitti_odometry/seq_${SEQ}/" --m_score $PV_RCNN_MIN_SCORE --m_classes Car,Pedestrian,Cyclist --m_topic /pvrcnn_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_pvrcnn" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_pvrcnn" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_pvrcnn"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes car,moving-car,bicyclist,person,moving-bicyclist,moving-person --sk_topic /sk_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test1_exp1_sk"

        # EXPERIMENT 2 ----------------------------------
        echo "EXPERIMENT 2 ----------------------------------"

        # 3D-MOOD
        echo "3D-MOOD"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --m_det_dir "${DATA_ROOT}/3D_MOOD_detections/kitti_odometry/CarPedCyc/seq_${SEQ}/" --m_score $MOOD_MIN_SCORE --m_classes Car,Pedestrian,Cyclist --m_topic /mood_filtered_clouds --m_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_mood" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_mood" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_mood"

        # PV-RCNN
        echo "PV-RCNN"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --m_det_dir "${DATA_ROOT}/openPCDet_detections/pv_rcnn/kitti_odometry/seq_${SEQ}/" --m_score $PV_RCNN_MIN_SCORE --m_classes Car,Pedestrian,Cyclist --m_topic /pvrcnn_filtered_clouds --m_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_pvrcnn" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_pvrcnn" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_pvrcnn"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes car,moving-car,bicyclist,person,moving-bicyclist,moving-person --sk_topic /sk_filtered_clouds --sk_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test1_exp2_sk"
    fi

    if echo "$TEST_SELECTOR" | grep -qE '\<(2|all)\>'; then
        # ---------------------- TEST 2 ----------------------------------
        echo "---------------------- TEST 2 ----------------------------------"

        # EXPERIMENT 1 ----------------------------------
        echo "EXPERIMENT 1 ----------------------------------"

        # PV-RCNN
        echo "PV-RCNN"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --m_det_dir "${DATA_ROOT}/openPCDet_detections/pv_rcnn/kitti_odometry/seq_${SEQ}/" --m_score $PV_RCNN_MIN_SCORE --m_classes Car,Pedestrian,Cyclist --m_topic /pvrcnn_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test2_exp1_pvrcnn" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test2_exp1_pvrcnn" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test2_exp1_pvrcnn"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes car,moving-car,bicyclist,person,moving-bicyclist,moving-person --sk_topic /sk_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test2_exp1_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test2_exp1_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test2_exp1_sk"

        # EXPERIMENT 2 ----------------------------------
        echo "EXPERIMENT 2 ----------------------------------"

        # PV-RCNN
        echo "PV-RCNN"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --m_det_dir "${DATA_ROOT}/openPCDet_detections/pv_rcnn/kitti_odometry/seq_${SEQ}/" --m_score $PV_RCNN_MIN_SCORE --m_classes Car,Pedestrian,Cyclist --m_topic /pvrcnn_filtered_clouds --m_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test2_exp2_pvrcnn" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test2_exp2_pvrcnn" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test2_exp2_pvrcnn"  

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes car,moving-car,bicyclist,person,moving-bicyclist,moving-person --sk_topic /sk_filtered_clouds --sk_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test2_exp2_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test2_exp2_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test2_exp2_sk"
    fi

    if echo "$TEST_SELECTOR" | grep -qE '\<(3|all)\>'; then
        # ---------------------- TEST 3 ----------------------------------
        echo "---------------------- TEST 3 ----------------------------------"

        # EXPERIMENT 1 ----------------------------------
        echo "EXPERIMENT 1 ----------------------------------"

        # 3D-MOOD
        echo "3D-MOOD"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --m_det_dir "${DATA_ROOT}/3D_MOOD_detections/kitti_odometry/CarPedCycBusTruckMotTrafVeg/seq_${SEQ}/" --m_score $MOOD_MIN_SCORE --m_classes "Car,Pedestrian,Cyclist,Bus,Truck,Motorbike,Traffic sign,Vegetation" --m_topic /mood_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test3_exp1_mood" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test3_exp1_mood" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test3_exp1_mood"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes car,moving-car,bicyclist,person,moving-bicyclist,moving-person,bicycle,bus,motorcycle,truck,moving-motorcyclist,moving-bus,moving-truck,other-vehicle,moving-other-vehicle,traffic-sign,vegetation --sk_topic /sk_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test3_exp1_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test3_exp1_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test3_exp1_sk"

        # EXPERIMENT 2 ----------------------------------
        echo "EXPERIMENT 2 ----------------------------------"

        # 3D-MOOD
        echo "3D-MOOD"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --m_det_dir "${DATA_ROOT}/3D_MOOD_detections/kitti_odometry/CarPedCycBusTruckMotTrafVeg/seq_${SEQ}/" --m_score $MOOD_MIN_SCORE --m_classes "Car,Pedestrian,Cyclist,Bus,Truck,Motorbike,Traffic sign,Vegetation" --m_topic /mood_filtered_clouds --m_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test3_exp2_mood" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test3_exp2_mood" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test3_exp2_mood"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes car,moving-car,bicyclist,person,moving-bicyclist,moving-person,bicycle,bus,motorcycle,truck,moving-motorcyclist,moving-bus,moving-truck,other-vehicle,moving-other-vehicle,traffic-sign,vegetation --sk_topic /sk_filtered_clouds --sk_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test3_exp2_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test3_exp2_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test3_exp2_sk"
    fi

    if echo "$TEST_SELECTOR" | grep -qE '\<(4|all)\>'; then
        # ---------------------- TEST 4 ----------------------------------
        echo "---------------------- TEST 4 ----------------------------------"

        # EXPERIMENT 1 ----------------------------------
        echo "EXPERIMENT 1 ----------------------------------"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes moving-car,moving-bicyclist,moving-person,moving-motorcyclist,moving-bus,moving-truck,moving-other-vehicle --sk_topic /sk_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test4_exp1_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test4_exp1_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test4_exp1_sk"

        # EXPERIMENT 2 ----------------------------------
        echo "EXPERIMENT 2 ----------------------------------"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes moving-car,moving-bicyclist,moving-person,moving-motorcyclist,moving-bus,moving-truck,moving-other-vehicle --sk_topic /sk_filtered_clouds --sk_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test4_exp2_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test4_exp2_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test4_exp2_sk"
    fi

    if echo "$TEST_SELECTOR" | grep -qE '\<(5|all)\>'; then
        # ---------------------- TEST 5 ----------------------------------
        echo "---------------------- TEST 5 ----------------------------------"

        # EXPERIMENT 1 ----------------------------------
        echo "EXPERIMENT 1 ----------------------------------"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes moving-car,moving-bicyclist,moving-person,moving-motorcyclist,moving-bus,moving-truck,moving-other-vehicle --sk_topic /sk_filtered_clouds -o "${BASE_DEST_DIR}/seq${SEQ}_test5_exp1_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test5_exp1_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test5_exp1_sk"

        # EXPERIMENT 2 ----------------------------------
        echo "EXPERIMENT 2 ----------------------------------"

        # SEMANTIC KITTI
        echo "SEMANTIC KITTI"
        ros2 run pc_filter dataset_pc_filter -p "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/velodyne/" --ts_file "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/times.txt" --fov_filter "${DATA_ROOT}/fov_calib_files/fov_calib_seq_${SEQ}.txt" --sk_lbl_dir "${DATASET_ROOT}/kitti/odometry/dataset/sequences/${SEQ}/labels/" --sk_classes moving-car,moving-bicyclist,moving-person,moving-motorcyclist,moving-bus,moving-truck,moving-other-vehicle --sk_topic /sk_filtered_clouds --sk_max_dist $MAX_DIST -o "${BASE_DEST_DIR}/seq${SEQ}_test5_exp2_sk" --no_confirm

        genz_icp_pipeline "${BASE_DEST_DIR}/seq${SEQ}_test5_exp2_sk" --config kitti.yaml
        move_and_clean "${BASE_DEST_DIR}/seq${SEQ}_test5_exp2_sk"
    fi

done

conda deactivate
echo "ODOMETRY POSES GENERATION COMPLETED!"
