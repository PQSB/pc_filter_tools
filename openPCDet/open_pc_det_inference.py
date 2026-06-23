import argparse
import glob
from pathlib import Path
import os

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=UserWarning)

import numpy as np
import torch

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils

from tqdm import tqdm

class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, ext='.bin'):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.root_path = root_path
        self.ext = ext
        data_file_list = glob.glob(str(root_path / f'*{self.ext}')) if self.root_path.is_dir() else [self.root_path]

        data_file_list.sort()
        self.sample_file_list = data_file_list

    def __len__(self):
        return len(self.sample_file_list)

    def __getitem__(self, index):
        if self.ext == '.bin':
            points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 4)
        elif self.ext == '.npy':
            points = np.load(self.sample_file_list[index])
        else:
            raise NotImplementedError

        input_dict = {
            'points': points,
            'frame_id': index,
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict


def export_3d_detections(out_dir, img_id, boxes3d, scores, class_ids, class_names):

    filepath = os.path.join(out_dir, f"{img_id}.txt")

    with open(filepath, "w") as f:
        n = len(boxes3d)

        if n == 0:
            # pbar.write(f"[WARN] No detections in image {img_id}")
            return

        for b3d, s, cat in zip(boxes3d, scores, class_ids):

            # b3d = [x, y, z, w, l, h, yaw]
            b = b3d.cpu().numpy()

            class_idx = int(cat.item()) - 1
            category = class_names[class_idx] if 0 <= class_idx < len(class_names) else f"Unknown({cat})"

            score = float(s.item())

            # 3D bounding box
            x, y, z, l, w, h, ry = b

            ry = np.arctan2(np.sin(ry), np.cos(ry))

            f.write(
                f"{category} "
                f"{x:.4f} {y:.4f} {z:.4f} "
                f"{w:.4f} {l:.4f} {h:.4f} {ry:.4f} "
                f"{score:.2f}\n"
            )
    return

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='cfgs/kitti_models/second.yaml',
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')
    parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')

    parser.add_argument("--out_det", type=str, required=True, help="path to store the txt with the detections")

    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)

    return args, cfg


def main():
    args, cfg = parse_config()
    logger = common_utils.create_logger()
    logger.info('-----------------Quick Demo of OpenPCDet-------------------------')
    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
        root_path=Path(args.data_path), ext=args.ext, logger=logger
    )
    logger.info(f'Total number of samples: \t{len(demo_dataset)}')

    class_names = cfg.CLASS_NAMES
    os.makedirs(args.out_det, exist_ok=True)

    model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda()
    model.eval()
    with torch.no_grad():
        # for idx, data_dict in enumerate(demo_dataset):
        for idx, data_dict in enumerate(tqdm(demo_dataset, total=len(demo_dataset), desc="openPCDet inference:")):
            # logger.info(f'Visualized sample index: \t{idx + 1}')
            data_dict = demo_dataset.collate_batch([data_dict])
            load_data_to_gpu(data_dict)
            pred_dicts, _ = model.forward(data_dict)

            img_id = Path(demo_dataset.sample_file_list[idx]).stem

            export_3d_detections(
                out_dir=args.out_det,
                img_id=img_id,
                boxes3d=pred_dicts[0]['pred_boxes'],
                scores=pred_dicts[0]['pred_scores'],
                class_ids=pred_dicts[0]['pred_labels'],
                class_names=class_names
            )

if __name__ == '__main__':
    main()
