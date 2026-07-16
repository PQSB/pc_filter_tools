"""Demo 3D-MOOD with KITTI dataset."""

import numpy as np
import torch
from PIL import Image

from vis4d.data.transforms.base import compose
from vis4d.data.transforms.normalize import NormalizeImages
from vis4d.data.transforms.resize import ResizeImages, ResizeIntrinsics
from vis4d.data.transforms.to_tensor import ToTensor
from vis4d.common.ckpt import load_model_checkpoint
from vis4d.op.fpp.fpn import FPN
# from vis4d.vis.image.functional import imshow_bboxes3d
from vis4d.vis.image.functional import draw_bbox3d

from opendet3d.data.transforms.pad import CenterPadImages, CenterPadIntrinsics
from opendet3d.data.transforms.resize import GenResizeParameters
from opendet3d.model.detect3d.grounding_dino_3d import GroundingDINO3D
from opendet3d.op.base.swin import SwinTransformer
from opendet3d.op.detect3d.grounding_dino_3d import (
    GroundingDINO3DCoder,
    GroundingDINO3DHead,
    RoI2Det3D,
    UniDepthHead,
)
from opendet3d.op.fpp.channel_mapper import ChannelMapper

from scipy.spatial.transform import Rotation as R

from pathlib import Path
import os

from tqdm import tqdm

import argparse

import cv2

import sys

from mood_utils import depth_2_cloud, get_cam2_2_lidar_matrix, load_kitti_matrix, load_intrinsics_from_yaml

def export_3d_detections(out_dir, img_id, boxes3d, scores, class_ids, cam2_2_lidar=None):

    filepath = os.path.join(out_dir, f"{img_id}.txt")

    with open(filepath, "w") as f:
        n = len(boxes3d)

        if n == 0:
            # pbar.write(f"[WARN] No detections in image {img_id}")
            return

        for b3d, s, cat in zip(boxes3d, scores, class_ids):

            # b3d = [x, y, z, w, l, h, yaw]
            b = b3d.cpu().numpy()

            category = str(cat)
            score = float(s.item())

            # 3D bounding box
            x, y, z, w, l, h, qw, qx, qy, qz = b

            if cam2_2_lidar is not None:
                R_det = R.from_quat([qx, qy, qz, qw]).as_matrix()

                det_2_cam2 = np.eye(4)

                det_2_cam2[:3, :3] = R_det
                det_2_cam2[:3, 3] = [x, y, z]
            
                det_2_lidar = cam2_2_lidar @ det_2_cam2

                x_l, y_l, z_l = det_2_lidar[:3, 3]

                R_lidar = det_2_lidar[:3, :3]

                ry_l = np.arctan2(R_lidar[1, 0], R_lidar[0, 0])
                ry_l = np.arctan2(np.sin(ry_l), np.cos(ry_l))

                x, y, z = x_l, y_l, z_l
                ry = ry_l

            else:
                ry = np.arctan2(2 * (qw * qy + qx * qz), 1 - 2 * (qy**2 + qz**2))
                # ry [-pi, pi]
                ry = np.arctan2(np.sin(ry), np.cos(ry))

            f.write(
                f"{category} "
                f"{x:.4f} {y:.4f} {z:.4f} "
                f"{w:.4f} {l:.4f} {h:.4f} {ry:.4f} "
                f"{score:.2f}\n"
            )
    return

def export_mood_point_cloud(out_dir, img_id, depth_img, intrinsics, pbar):
    filepath = os.path.join(out_dir, f"{img_id}.bin")

    cloud = depth_2_cloud(depth_img, intrinsics)
    cloud.astype(np.float32).tofile(filepath)

    return

def get_3d_mood_swin_base(
    max_per_image: int = 100, score_thres: float = 0.1
) -> GroundingDINO3D:
    """Get the config of Swin-Base."""
    basemodel = SwinTransformer(
        convert_weights=True,
        pretrain_img_size=384,
        embed_dims=128,
        depths=[2, 2, 18, 2],
        num_heads=[4, 8, 16, 32],
        window_size=12,
        drop_path_rate=0.3,
        out_indices=(0, 1, 2, 3),
    )

    neck = ChannelMapper(
        in_channels=[256, 512, 1024],
        out_channels=256,
        num_outs=4,
        kernel_size=1,
        norm="GroupNorm",
        num_groups=32,
        activation=None,
        bias=True,
    )

    depth_fpn = FPN(
        in_channels_list=[128, 256, 512, 1024],
        out_channels=256,
        extra_blocks=None,
        start_index=0,
    )

    depth_head = UniDepthHead(input_dims=[256, 256, 256, 256])

    box_coder = GroundingDINO3DCoder()

    bbox3d_head = GroundingDINO3DHead(box_coder=box_coder)

    roi2det3d = RoI2Det3D(
        nms=True,
        class_agnostic_nms=True,
        max_per_img=max_per_image,
        score_threshold=score_thres,
    )

    return GroundingDINO3D(
        basemodel=basemodel,
        neck=neck,
        bbox3d_head=bbox3d_head,
        roi2det3d=roi2det3d,
        fpn=depth_fpn,
        depth_head=depth_head,
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export the detections and point clouds resulting from the inference using 3D-MOOD")
    parser.add_argument("--input", type=str, required=True, help="Path to the input images directory")
    parser.add_argument("--intrinsics", type=str, required=True, help="Path to the camera intrinsics file (kitti calib file format of yaml)")
    parser.add_argument("--out_detections", type=str, help="Path in which to store the detections")
    parser.add_argument("--out_pointcloud", type=str, help="Path in which to store the generated point clouds")
    parser.add_argument("--out_images", type=str, help="Path in which to store the images with the 3D bounding boxes")
    parser.add_argument("--depth_images", type=str, help="Path in which to store the depth images calculated by 3D-MOOD")
    parser.add_argument("--prompt", type=str, default="chair.table.person.bin", help="Classes to detect, separated by dots (e.g., Car.Cat)")
    parser.add_argument("--lidar_ref", type=str, help="Path to the file containing the lidar2cam matrix (export detections in LIDAR coordinates)")

    args = parser.parse_args()

    cam02velo = None

    if args.lidar_ref:
        if args.out_detections:
            velo2cam0 = load_kitti_matrix(args.lidar_ref, "Tr:")
            if velo2cam0 is None:
                print("velo2cam reading failed")
                sys.exit(1)
            velo2cam0 = np.vstack((velo2cam0, np.array([0., 0., 0., 1.])))
            cam02velo = np.linalg.inv(velo2cam0)
        else:
            parser.error("--lidar_ref requires --out_detections")

    """Demo."""
    device = "cuda" if torch.cuda.is_available() else "cpu"

    img_root = Path(args.input)

    detect = False
    pointcloud = False
    images = False
    depth_images = False

    if args.out_detections:
        out_det_root = Path(args.out_detections)
        out_det_root.mkdir(exist_ok=True)
        detect = True

    if args.out_pointcloud:
        out_pc_root = Path(args.out_pointcloud)
        out_pc_root.mkdir(exist_ok=True)
        pointcloud = True

    if args.out_images:
        out_img_root = Path(args.out_images)
        out_img_root.mkdir(exist_ok=True)
        images = True

    if args.depth_images:
        out_depth_img_root = Path(args.depth_images)
        out_depth_img_root.mkdir(exist_ok=True)
        depth_images = True 

    # Load the Model just once
    model = get_3d_mood_swin_base().to(device)

    load_model_checkpoint(
        model,
        weights="https://huggingface.co/RoyYang0714/3D-MOOD/resolve/main/gdino3d_swin-b_120e_omni3d_834c97.pt",
        rev_keys=[(r"^model\.", ""), (r"^module\.", "")],
    )

    model.eval()

    # Prompt for G-DINO
    text_prompts = args.prompt

    input_texts = text_prompts.split(".")

    preprocess_transforms = compose(
        transforms=[
            GenResizeParameters(shape=(800, 1333)),
            ResizeImages(),
            ResizeIntrinsics(),
            NormalizeImages(),
            CenterPadImages(stride=1, shape=(800, 1333), update_input_hw=True),
            CenterPadIntrinsics(),
        ]
    )

    # Convert to Tensor
    to_tensor = ToTensor()

    img_paths = sorted(list(img_root.glob("*.png")) + list(img_root.glob("*.jpg")))

    if not img_paths:
        print(f"\n[ERROR] No images found: {img_root.resolve()}")
        print("Extensions supported: .png, .jpg")
        sys.exit(1)

    # Mapping for 3D bounding boxes
    class_id_mapping = {i: text for i, text in enumerate(input_texts)}

    cam2_2_lidar = None
    if cam02velo is not None:
        P2 = load_kitti_matrix(args.intrinsics, "P2:")
        if P2 is None:
            print("P2 reading failed")
            sys.exit(1)

        cam2_2_lidar = get_cam2_2_lidar_matrix(P2, cam02velo)
        intrinsics = P2[:, :3]

    else:
        intrinsics = load_intrinsics_from_yaml(args.intrinsics)

    pbar = tqdm(img_paths, desc="3D-MOOD Inference")

    for img_path in pbar:
        # Get the image name without the extension
        img_id = img_path.stem

        image = np.array(Image.open(img_path)).astype(np.float32)[
                None, ...
            ]

        data_dict = {
            "images": image,
            "original_images": image,
            "input_hw": (image.shape[1], image.shape[2]),
            "original_hw": (image.shape[1], image.shape[2]),
            "intrinsics": intrinsics,
            "original_intrinsics": intrinsics,
        }

        data = preprocess_transforms([data_dict])[0]
        data = to_tensor([data])[0]

        # Run predict
        with torch.no_grad():
            boxes, boxes3d, scores, class_ids, depth_maps, categories = model(
                images=data["images"].to(device),
                input_hw=[data["input_hw"]],
                original_hw=[data["original_hw"]],
                intrinsics=data["intrinsics"].to(device)[None],
                padding=[data["padding"]],
                input_texts=[input_texts],
            )

        if detect:
            export_3d_detections(
                out_det_root, img_id, boxes3d[0].cpu(),scores[0].cpu(), categories[0], cam2_2_lidar)

        if pointcloud:
            depth_img = depth_maps[0].cpu().numpy()

            export_mood_point_cloud(
                out_pc_root, img_id, depth_img, intrinsics, pbar)

        if images:
            out_img_path = os.path.join(out_img_root, f"{img_id}.png")

            # Generate the bounding boxes of the detecions in the image
            canvas = draw_bbox3d(
                image=data["original_images"].cpu(),
                boxes3d=[b.cpu() for b in boxes3d],
                intrinsics=data["original_intrinsics"].cpu().numpy(),
                scores=[s.cpu() for s in scores],
                class_ids=[c.cpu() for c in class_ids],
                class_id_mapping=class_id_mapping,
                n_colors=len(class_id_mapping)
            )

            # Store the image
            canvas.save_to_disk(out_img_path)

        if depth_images:
            out_depth_img_path = os.path.join(out_depth_img_root, f"{img_id}.png")

            depth_img = depth_maps[0].squeeze().cpu().numpy()
            depth_in_mm = depth_img * 1000.0

            depth_in_mm = np.clip(depth_in_mm, 0, 65535) 

            # uint16 to export it in mm
            depth_in_mm = depth_in_mm.astype(np.uint16)

            cv2.imwrite(out_depth_img_path, depth_in_mm)
