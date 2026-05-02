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
from vis4d.vis.image.functional import imshow_bboxes3d

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

from pathlib import Path
import os

from tqdm import tqdm

import argparse

from utils import load_intrinsics_from_yaml, depth_2_cloud

def export_3d_detections(out_dir, img_id, boxes3d, scores, class_ids, pbar):

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
            ry = np.arctan2(2 * (qw * qy + qx * qz), 1 - 2 * (qy**2 + qz**2))
            # ry [-pi, pi]
            ry = np.arctan2(np.sin(ry), np.cos(ry))

            f.write(
                f"{category} "
                f"{x:.4f} {y:.4f} {z:.4f} "
                f"{w:.4f} {l:.4f} {h:.4f} {ry:.4f}"
                f"{score:.2f}\n"
            )

    # pbar.write(f"[OK] Image {img_id} detections exported to {filepath}")
    return

def export_mood_point_cloud(out_dir, img_id, depth_img, intrinsics, pbar):
    filepath = os.path.join(out_dir, f"{img_id}.bin")

    cloud = depth_2_cloud(depth_img, intrinsics)
    cloud.astype(np.float32).tofile(filepath)

    # pbar.write(f"[OK] Point cloud from {img_id} exported to {filepath}")
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
    parser = argparse.ArgumentParser(description="Export the detections and point clouds resulting from the inference using 3dmood")
    parser.add_argument("--input", type=str, required=True, help="Path to the input images directory")
    parser.add_argument("--intrinsics", type=str, required=True, help="Path to the camera intrinsics file")
    parser.add_argument("--out_detections", type=str, help="Path in which to store the detections")
    parser.add_argument("--out_pointcloud", type=str, help="Path in which to store the generated point clouds")
    parser.add_argument("--prompt", type=str, default="chair.table.person.bin", help="Classes to detect, separated by dots (e.g., Car.Cat)")
    parser.add_argument("--weights", type=str, required=True, help="Path to the model weights")

    args = parser.parse_args()

    """Demo."""
    device = "cuda" if torch.cuda.is_available() else "cpu"

    img_root = Path(args.input)

    detect = False
    pointcloud = False

    if args.out_detections:
        out_det_root = Path(args.out_detections)
        out_det_root.mkdir(exist_ok=True)
        detect = True

    if args.out_pointcloud:
        out_pc_root = Path(args.out_pointcloud)
        out_pc_root.mkdir(exist_ok=True)
        pointcloud = True

    # Load the Model just once
    model = get_3d_mood_swin_base().to(device)

    load_model_checkpoint(
        model,
        weights=args.weights,
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

    img_paths = sorted(list(img_root.glob("*.png")))

    # Mapping for 3D bounding boxes
    class_id_mapping = {i: text for i, text in enumerate(input_texts)}

    intrinsics = load_intrinsics_from_yaml(args.intrinsics)

    pbar = tqdm(img_paths, desc="3D-MOOD Inference")

    for img_path in pbar:
        # Get the image name without the extension
        img_id = img_path.stem

        # pbar.write(f"Processing image {img_id}")

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
                out_det_root, img_id, boxes3d[0].cpu(),scores[0].cpu(), categories[0], pbar)

        if pointcloud:
            depth_img = depth_maps[0].cpu().numpy()

            export_mood_point_cloud(
                out_pc_root, img_id, depth_img, intrinsics, pbar)
