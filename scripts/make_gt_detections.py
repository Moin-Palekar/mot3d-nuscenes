"""Writes the val ground-truth boxes as a nuScenes detection result file.

Feeding perfect detections to the tracker is a sanity gate: a correct pipeline
should score close to 1.0 AMOTA on them. A low score points at a bug in the
tracker or the I/O, not at the detector.

Usage: python3 scripts/make_gt_detections.py [output.json]
Default output: results/gt_detections.json
"""

import json
import math
import os
import sys

from nuscenes import NuScenes
from nuscenes.eval.tracking.utils import category_to_tracking_name
from nuscenes.utils.splits import create_splits_scenes


def main() -> None:
    output_path = sys.argv[1] if len(sys.argv) > 1 else "results/gt_detections.json"

    nusc = NuScenes(version="v1.0-trainval", dataroot="data", verbose=False)
    val_scene_names = set(create_splits_scenes()["val"])

    results = {}
    num_boxes = 0
    for scene in nusc.scene:
        if scene["name"] not in val_scene_names:
            continue
        sample_token = scene["first_sample_token"]
        while sample_token:
            sample = nusc.get("sample", sample_token)
            boxes = []
            for ann_token in sample["anns"]:
                ann = nusc.get("sample_annotation", ann_token)
                tracking_name = category_to_tracking_name(ann["category_name"])
                if tracking_name is None:
                    continue  # not one of the 7 tracking classes
                vx, vy = nusc.box_velocity(ann_token)[:2]
                if not (math.isfinite(vx) and math.isfinite(vy)):
                    vx, vy = 0.0, 0.0  # JSON has no NaN; seen once, so no velocity
                boxes.append({
                    "sample_token": sample_token,
                    "translation": ann["translation"],
                    "size": ann["size"],
                    "rotation": ann["rotation"],
                    "velocity": [float(vx), float(vy)],
                    "detection_name": tracking_name,
                    "detection_score": 1.0,
                    "attribute_name": "",
                })
            results[sample_token] = boxes
            num_boxes += len(boxes)
            sample_token = sample["next"]

    submission = {
        "meta": {
            "use_camera": False,
            "use_lidar": True,
            "use_radar": False,
            "use_map": False,
            "use_external": False,
        },
        "results": results,
    }
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(submission, f)
    print(f"frames: {len(results)}")
    print(f"boxes:  {num_boxes}")
    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()