#!/usr/bin/env python3
# =============================================================================
# 04_extract_coco.py — COCO train2017 → YOLO 추출 (person + real 과일 보조)
#
# classes.yaml의 coco_map(person→6, apple→0, orange→2, banana→3)을 사용해
# 해당 물체가 나오는 이미지를 뽑아 YOLO format으로 변환한다.
# person은 손/팔만 보이는 이미지도 포함(작은 bbox 허용) — 데모 중 손 진입 대응.
#
# 용법 (데스크톱, venv):
#   python3 04_extract_coco.py
#
# 입력:  ~/capstone_training/assets/coco/{train2017, annotations}
# 출력:  ~/capstone_training/datasets/coco_yolo/{images,labels}/
#
# 규칙:
# - 이미지에 우리 class가 여러 개면 전부 라벨링 (같은 세트 안의 unlabeled
#   positive 방지)
# - iscrowd(군중 덩어리 박스) annotation이 우리 class에 있는 이미지는 통째로
#   제외 — 개별 박스가 없는 사람이 unlabeled positive로 남는 것을 방지
# - class별 이미지 수 상한: classes.yaml caps (person 8000, 과일 각 1500)
# =============================================================================
import argparse
import os
import random
import shutil
from collections import Counter

import yaml


def parse_args():
    home = os.path.expanduser("~")
    p = argparse.ArgumentParser()
    p.add_argument("--coco-root",
                   default=os.path.join(home, "capstone_training", "assets", "coco"))
    p.add_argument("--out",
                   default=os.path.join(home, "capstone_training", "datasets", "coco_yolo"))
    p.add_argument("--split", default="train2017")
    p.add_argument("--min-box-px", type=int, default=8)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-person", type=int, default=None,
                   help="기본값: classes.yaml caps.coco_person_images")
    p.add_argument("--max-fruit-each", type=int, default=None,
                   help="기본값: classes.yaml caps.coco_fruit_images_each")
    return p.parse_args()


def load_class_config():
    cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "config", "classes.yaml")
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    return cfg


def main():
    args = parse_args()
    random.seed(args.seed)
    cfg = load_class_config()
    coco_map = dict(cfg["coco_map"])          # {"person": 6, "apple": 0, ...}
    names = {int(k): v for k, v in cfg["names"].items()}
    caps = cfg.get("caps", {})
    max_person = args.max_person or int(caps.get("coco_person_images", 8000))
    max_fruit = args.max_fruit_each or int(caps.get("coco_fruit_images_each", 1500))

    ann_path = os.path.join(args.coco_root, "annotations",
                            f"instances_{args.split}.json")
    img_dir = os.path.join(args.coco_root, args.split)
    if not os.path.exists(ann_path):
        raise SystemExit(f"annotation 없음: {ann_path} — unzip 확인")
    if not os.path.isdir(img_dir):
        raise SystemExit(f"이미지 폴더 없음: {img_dir} — unzip 확인")

    from pycocotools.coco import COCO  # (로드에 수십 초 걸릴 수 있음)
    print(f"[coco] annotation 로드 중... ({ann_path})")
    coco = COCO(ann_path)

    # COCO category name → our class id
    cat_ids = coco.getCatIds(catNms=list(coco_map.keys()))
    cats = coco.loadCats(cat_ids)
    catid_to_ours = {c["id"]: coco_map[c["name"]] for c in cats
                     if c["name"] in coco_map}
    person_cat_ids = [c["id"] for c in cats if c["name"] == "person"]
    fruit_cat_ids = [c["id"] for c in cats if c["name"] != "person"]

    # 후보 이미지: class별 상한을 고려해 과일 먼저(희소) → person 순으로 선정
    selected = {}  # img_id → set(our class ids in it)
    per_class_imgs = Counter()

    def try_add(img_id, cat_id):
        ours = catid_to_ours[cat_id]
        cap = max_person if ours == 6 else max_fruit
        if per_class_imgs[ours] >= cap:
            return
        if img_id in selected:
            if ours not in selected[img_id]:
                selected[img_id].add(ours)
                per_class_imgs[ours] += 1
            return
        selected[img_id] = {ours}
        per_class_imgs[ours] += 1

    for cat_id in fruit_cat_ids:
        ids = coco.getImgIds(catIds=[cat_id])
        random.shuffle(ids)
        for i in ids:
            try_add(i, cat_id)
    for cat_id in person_cat_ids:
        ids = coco.getImgIds(catIds=[cat_id])
        random.shuffle(ids)
        for i in ids:
            try_add(i, cat_id)

    os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "labels"), exist_ok=True)

    inst_count = Counter()
    n_saved = 0
    n_skipped_crowd = 0
    target_cat_ids = set(catid_to_ours.keys())

    for img_id in selected:
        info = coco.loadImgs([img_id])[0]
        w, h = info["width"], info["height"]
        anns = coco.loadAnns(coco.getAnnIds(imgIds=[img_id]))

        # 우리 class에 iscrowd가 있으면 이미지 제외 (unlabeled positive 방지)
        if any(a["category_id"] in target_cat_ids and a.get("iscrowd", 0) == 1
               for a in anns):
            n_skipped_crowd += 1
            continue

        lines = []
        for a in anns:
            if a["category_id"] not in target_cat_ids or a.get("iscrowd", 0) == 1:
                continue
            x, y, bw, bh = a["bbox"]
            # 이미지 경계로 clamp
            x = max(0.0, x); y = max(0.0, y)
            bw = min(bw, w - x); bh = min(bh, h - y)
            if bw < args.min_box_px or bh < args.min_box_px:
                continue
            cls = catid_to_ours[a["category_id"]]
            xc = (x + bw / 2.0) / w
            yc = (y + bh / 2.0) / h
            lines.append(f"{cls} {xc:.6f} {yc:.6f} {bw / w:.6f} {bh / h:.6f}")
            inst_count[cls] += 1
        if not lines:
            continue

        src = os.path.join(img_dir, info["file_name"])
        if not os.path.exists(src):
            continue
        stem = "coco_" + os.path.splitext(info["file_name"])[0]
        dst = os.path.join(args.out, "images",
                           stem + os.path.splitext(info["file_name"])[1])
        if not os.path.exists(dst):
            try:
                os.link(src, dst)
            except OSError:
                shutil.copy2(src, dst)
        with open(os.path.join(args.out, "labels", stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        n_saved += 1

    print(f"\n== COCO 추출 완료: 이미지 {n_saved}장 "
          f"(crowd 제외 {n_skipped_crowd}장) ==")
    for cls in sorted(inst_count):
        print(f"  {names[cls]:<15} 인스턴스 {inst_count[cls]:>6}")
    print(f"출력: {args.out}")


if __name__ == "__main__":
    main()
