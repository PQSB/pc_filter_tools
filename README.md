# pc_filter_tools

A collection of utilities for extracting, synchronizing, processing, and filtering LiDAR point cloud data. The repository includes scripts for input data preparation, ROS 2 bag processing, 3D object detection, field-of-view filtering, and experimental evaluation. These tools are intended to support the software provided by <https://github.com/PQSB/pc_filter>

---

# Repository Structure

| Directory / File                   | Description                                                                              |
| ---------------------------------- | ---------------------------------------------------------------------------------------- |
| `3D_MOOD/`                         | Utilities for running inference with the 3D-MOOD model and related helper functions.     |
| `data_extraction_scripts/`         | Scripts for extracting and synchronizing data from ROS 2 bags.                           |
| `experiment/`                      | Experimental resources, including scripts, results, and evaluation tools.                |
| `openPCDet/`                       | Configuration files and inference scripts for OpenPCDet.                                 |
| `prepare_fov_filter_calib_file.py` | Utility to generate calibration files for field-of-view filtering.                       |

---

# Main Scripts

## 3D_MOOD/mood_inference.py

### Purpose

Runs 3D-MOOD inference on input images to detect user-defined object classes and estimate their 3D geometry. The script can export 3D detections, reconstructed point clouds, estimated depth maps, and the input images annotated with the projected 3D bounding boxes. Optionally, detections can also be exported in the LiDAR coordinate frame by providing a `lidar2cam` calibration file.

#### Inputs

- **Input images (`--input`)**: Directory containing the images to be processed.

- **Camera intrinsics (`--intrinsics`)**: Camera calibration file in **KITTI calibration** or **YAML** (generated with *export_intrinsics.py*) format containing the intrinsic parameters required by 3D-MOOD.

- **LiDAR reference (`--lidar_ref`, optional)**: Path to a KITTI-style calibration file containing the `Tr:` transformation matrix (LiDAR-to-camera). When provided, detections are exported in the LiDAR coordinate frame. This option requires `--out_detections`.

- **Prompt (`--prompt`, optional)**: Dot-separated list of object classes to detect (e.g., `chair.table.person`).

#### Outputs

- 3D detections.

- Reconstructed point clouds.

- Estimated depth maps.

- Input images annotated with projected 3D bounding boxes.

### Usage

```bash
python 3D_MOOD/mood_inference.py [OPTIONS]
```

### Example

```bash
python mood_inference.py \
    --input data/images \
    --intrinsics calibration/camera_intrinsics.yaml \
    --lidar_ref calibration/lidar2cam.txt \
    --out_detections results/detections \
    --out_pointcloud results/pointclouds \
    --out_images results/annotated_images \
    --depth_images results/depth_maps \
    --prompt "chair.table.person"
```

---

## `data_extraction_scripts`

This directory contains the preprocessing utilities used to convert raw ROS 2 bag recordings into synchronized input data ready for point cloud filtering and evaluation. The scripts can be executed independently, although they are typically used as part of the following workflow:

1. Synchronize the selected sensor topics.
2. Generate a synchronized ROS 2 bag.
3. Extract the camera intrinsic calibration parameters.
4. Export the required sensor data into the required format.

### `sync_topics.py`

Synchronizes multiple ROS 2 topics recorded at different frequencies using a reference topic (typically the LiDAR stream). The script generates a CSV file containing the temporal correspondences between the selected topics, as well as a `times.txt` file with the timestamps of the reference topic for later stages of the pipeline.

#### Inputs
- **bag_path:** Input ROS 2 bag.

- **base_topic:** Reference topic used to synchronize the remaining topics (typically the slowest sensor).

- **target_topics:** List of topics to synchronize with the reference topic.

### Outputs
- Synchronization CSV relating the matched messages across topics.

- times.txt file containing the timestamps of the reference topic.

**Example**

```bash
python sync_topics.py \
    --bag_path input_rosbag \
    --base_topic /lidar_topic \
    --target_topics /camera/image /camera/depth \
    --out_times times.txt \
    --out_sync_file sync.csv
```

### `gen_sync_rosbag.py`

Creates a synchronized ROS bag by keeping only the messages selected during the synchronization stage. The resulting bag contains only aligned sensor data, making it suitable for processing and evaluation.

**Example**

```bash
python gen_sync_rosbag.py \
    --input_bag input_rosbag \
    --sync_file sync.csv \
    --out_bag out_sync_rosbag
```

### `export_intrinsics.py`

Extracts the intrinsic camera calibration parameters from a ROS 2 bag topic and stores them in a YAML configuration file. The exported calibration can be used by other scripts, such as *export_topic_msgs.py*, to reconstruct point clouds from depth images.

#### Inputs
- **bag:** Input ROS bag.

- **topic:** Camera information topic (sensor_msgs/CameraInfo) containing the intrinsic calibration.

- **output:** Output YAML file.

#### Outputs
- YAML file containing the camera intrinsic matrix.

**Example**

```bash
python export_intrinsics.py \
    --bag input_rosbag \
    --topic /camera/camera_info \
    --output camera_intrinsics.yaml
```

### `export_topic_msgs.py`

Exports synchronized sensor data into a structured directory layout suitable for downstream applications. Depending on the selected topics, the script can export LiDAR point clouds, camera images, depth images and optionally generate virtual point clouds from depth maps using the depth camera calibration produced by `export_intrinsics.py`.

#### Inputs

- **bag_path:** Input synchronized ROS bag.

- **out_root_dir:** Root directory where the exported data will be stored.

- **lidar (optional):** LiDAR topic and output directory in the format /topic:directory_name.

- **images (optional):** Image topic and output directory in the format /topic:directory_name.

- **depth_img (optional):** Depth image topic and output directory in the format /topic:directory_name.

- **depth2cloud (optional):** Depth image topic and output directory used to export point clouds reconstructed from depth images, specified as /topic:directory_name.

- **depth_k (required when using --depth2cloud):** YAML file containing the camera intrinsic matrix generated by export_intrinsics.py.
Outputs

#### Outputs

Depending on the selected options, the script exports:

- LiDAR point clouds.

- Camera images.

- Depth images.

- Point clouds reconstructed from depth images.

Each data type is stored in its corresponding output directory under --out_root_dir.

**Example**

```bash
python export_topic_msgs.py \
    --bag_path input_rosbag \
    --out_root_dir root_dir \
    --lidar /lidar_topic:lidar \
    --depth_img /depth_img_topic:depth_images \
    --images /img_topic:images \
    --depth2cloud /depth_img_topic:point_clouds_from_depth_img
    --depth_k depth_img_K.yaml
```

---

## experiment

Experimental assets, including scripts for dataset conversion, experiment execution, result generation, and performance evaluation and analysis.

### kitti2ros.py

Converts KITTI dataset sequences ground truth into ROS 2 compatible format. The script transforms the pose sequence while preserving the original trajectory, allowing KITTI ground-truth files to be used directly with ROS-based evaluation tools.

#### Inputs

- **`input_file`**: KITTI odometry ground-truth file (e.g., `00.txt`).

#### Outputs

- Converted odometry file in ROS 2 coordinates.

#### Usage

```bash
python kitti2ros.py [OPTIONS]
```

#### Example

```bash
python kitti2ros.py \
    00.txt \
    00_ros.txt
```


### `gen_odometry_poses.sh`

Automates the generation of estimated odometry trajectories for the experimental evaluation.


### `gen_evo_results_csv.sh`

Automates the evaluation of all experiments using the **evo** toolkit and generates a CSV file containing the resulting performance metrics.


### `write_evo_results.py`

Parses the evaluation results produced by **evo** (`evo_ape` and `evo_rpe`) and updates the corresponding CSV file with the metrics associated with each experiment.

---

## openPCDet/open_pc_det_inference.py

### Purpose

Runs inference using OpenPCDet on LiDAR point clouds.

### Usage

```bash
python open_pc_det_inference.py [OPTIONS]
```

### Example

```bash
python open_pc_det_inference.py \
    --cfg_file configs/example.yaml \
    --data_path data/
```

---

## prepare_fov_filter_calib_file.py

### Purpose

Generates calibration files required for field-of-view filtering of point cloud data.

### Usage

```bash
python prepare_fov_filter_calib_file.py [OPTIONS]
```

---

# Requirements

The repository contains utilities based on Python together with ROS-related tools and third-party frameworks such as OpenPCDet and 3D-MOOD. Please refer to each component for its specific dependencies.

---

# License

This project is distributed under the terms of the LICENSE file included in this repository.
