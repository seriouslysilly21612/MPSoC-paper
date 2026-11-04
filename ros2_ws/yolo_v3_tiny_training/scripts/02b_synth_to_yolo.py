#!/usr/bin/env python3
# =============================================================================
# 02b_synth_to_yolo.py — synthetic COCO 출력 → YOLO format 변환 + 집계
#
# 용법 (데스크톱, 일반 python):
#   python3 02b_synth_to_yolo.py                # 변환 + class별 인스턴스 수 출력
#   python3 02b_synth_to_yolo.py --count-only   # 변환 없이 현재까지 수량만 확인
#
# 입력:  ~/capstone_training/synth/batch_*/coco_data/coco_annotations.json
# 출력:  ~/capstone_training/datasets/synth_yolo/{images,labels}/
#        (YOLO txt: "class_id xc yc w h" 정규화 좌표, class_id는 classes.yaml 기준)
#
# - 렌더링의 category_id는 1-base(class+1) → 여기서 -1 하여 0-base로 변환
# - 가림 등으로 너무 작아진 bbox(기본 8px 미만)는 라벨에서 제외
# - 물체가 하나도 안 보이는 이미지도 negative(빈 라벨)로 보존
# =============================================================================
import argparse
import glob
import json
import os
import shutil
from collections import Counter

CLASS_NAMES = {0: "apple", 1: "orange", 2: "banana",
               3: "tennis_ball", 4: "mustard_bottle", 5: "person"}


def parse_args():
    home = os.path.expanduser("~")
    p = argparse.ArgumentParser()
    p.add_argument("--synth-dir", default=os.path.join(home, "capstone_training", "synth"))
    p.add_argument("--out", default=os.path.join(home, "capstone_training",
                                                 "datasets", "synth_yolo"))
    p.add_argument("--min-box-px", type=int, default=8,
                   help="이 픽셀보다 작은 변의 bbox는 제외")
    p.add_argument("--count-only", action="store_true",
                   help="변환 없이 인스턴스 수만 집계")
    return p.parse_args()


def link_or_copy(src, dst):
    try:
        os.link(src, dst)  # 같은 파티션이면 hardlink (용량 절약)
    except OSError:
        shutil.copy2(src, dst)


def main():
    args = parse_args()
    json_paths = sorted(glob.glob(os.path.join(
        args.synth_dir, "**", "coco_data", "coco_annotations.json"), recursive=True))
    if not json_paths:
        raise SystemExit(f"coco_annotations.json 없음: {args.synth_dir} — 02 렌더링 먼저")

    if not args.count_only:
        os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
        os.makedirs(os.path.join(args.out, "labels"), exist_ok=True)

    inst_count = Counter()
    dropped_small = 0
    n_images = 0
    n_negatives = 0

    for jp in json_paths:
        coco_dir = os.path.dirname(jp)
        # batch 이름을 파일명 prefix로 사용 (batch 간 파일명 충돌 방지)
        batch_tag = os.path.basename(os.path.dirname(coco_dir))
        with open(jp) as f:
            coco = json.load(f)

        anns_by_image = {}
        for ann in coco.get("annotations", []):
            anns_by_image.setdefault(ann["image_id"], []).append(ann)

        for img in coco.get("images", []):
            w, h = img["width"], img["height"]
            lines = []
            for ann in anns_by_image.get(img["id"], []):
                cat = ann["category_id"]
                if cat <= 0:  # background/distractor
                    continue
                cls = cat - 1
                bx, by, bw, bh = ann["bbox"]
                if bw < args.min_box_px or bh < args.min_box_px:
                    dropped_small += 1
                    continue
                xc = (bx + bw / 2.0) / w
                yc = (by + bh / 2.0) / h
                lines.append(f"{cls} {xc:.6f} {yc:.6f} {bw / w:.6f} {bh / h:.6f}")
                inst_count[cls] += 1

            n_images += 1
            if not lines:
                n_negatives += 1
            if args.count_only:
                continue

            src_img = os.path.join(coco_dir, img["file_name"])
            stem = f"{batch_tag}_{os.path.splitext(os.path.basename(img['file_name']))[0]}"
            ext = os.path.splitext(img["file_name"])[1]
            dst_img = os.path.join(args.out, "images", stem + ext)
            if not os.path.exists(dst_img):
                link_or_copy(src_img, dst_img)
            with open(os.path.join(args.out, "labels", stem + ".txt"), "w") as f:
                f.write("\n".join(lines) + ("\n" if lines else ""))

    print(f"\n== synthetic 집계 ({len(json_paths)} batch, 이미지 {n_images}장, "
          f"negative {n_negatives}장, 작아서 제외한 bbox {dropped_small}개) ==")
    target = 3000  # classes.yaml: synthetic_target_instances_per_class
    for cls in sorted(CLASS_NAMES):
        if cls == 6:
            continue  # person은 synthetic 대상 아님
        n = inst_count[cls]
        bar = "OK " if n >= target else "   "
        print(f"  [{bar}] {CLASS_NAMES[cls]:<15} {n:>6} / {target}")
    if not args.count_only:
        print(f"\n출력: {args.out}")


if __name__ == "__main__":
    main()
