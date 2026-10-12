"""Write an empty but valid nuScenes tracking submission for the val split.

Every val sample token is present with an empty list of boxes. Scoring this
file proves the official scorer accepts our submission format before any
tracker exists, so a bad score later cannot be a file-format problem.
"""

import argparse
import json
import os

from nuscenes.utils.splits import create_splits_scenes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tables", default="data/v1.0-trainval",
                        help="Folder containing scene.json and sample.json")
    parser.add_argument("--out", default="results/empty_submission.json",
                        help="Where to write the submission")
    args = parser.parse_args()

    with open(os.path.join(args.tables, "scene.json")) as f:
        scenes = json.load(f)
    with open(os.path.join(args.tables, "sample.json")) as f:
        samples = json.load(f)

    # The official list of val scene names, as defined by the devkit itself.
    val_scene_names = set(create_splits_scenes()["val"])
    val_scene_tokens = {s["token"] for s in scenes if s["name"] in val_scene_names}
    val_sample_tokens = [s["token"] for s in samples if s["scene_token"] in val_scene_tokens]

    submission = {
        "meta": {
            "use_camera": False,
            "use_lidar": True,
            "use_radar": False,
            "use_map": False,
            "use_external": False,
        },
        # Every val frame present, each with no boxes.
        "results": {token: [] for token in val_sample_tokens},
    }
    
    # The scorer decides the box type ("tracking" or "detection") by looking at
    # the first box it finds. With no boxes at all it cannot decide and stops.
    # So add one placeholder box at the map origin (0, 0, 0). Real frames are
    # hundreds of metres or more from there, beyond the ~50 m scoring range, so
    # the scorer filters it out before scoring anything.
    first_token = val_sample_tokens[0]
    submission["results"][first_token].append({
        "sample_token": first_token,
        "translation": [0.0, 0.0, 0.0],
        "size": [1.0, 1.0, 1.0],
        "rotation": [1.0, 0.0, 0.0, 0.0],
        "velocity": [0.0, 0.0],
        "tracking_id": "placeholder",
        "tracking_name": "car",
        "tracking_score": 0.5,
    })

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(submission, f)

    print(f"val scenes: {len(val_scene_tokens)}  val samples: {len(val_sample_tokens)}")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()