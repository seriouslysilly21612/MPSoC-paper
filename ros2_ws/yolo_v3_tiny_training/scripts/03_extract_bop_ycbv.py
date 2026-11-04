#!/usr/bin/env python3
# =============================================================================
# 03_extract_bop_ycbv.py — BOP ycbv(YCB-Video) → YOLO 추출
#
# mustard_bottle(BOP obj 5)과 banana(BOP obj 10)가 등장하는 real frame만 골라
# YOLO format으로 변환한다. 같은 frame에 있는 나머지 YCB 물체(캔, 드릴 등)는
# 우리 class가 아니므로 라벨 없이 그대로 두어 자연스러운 distractor가 된다.
#
# 용법 (데스크톱, venv):
#   python3 03_extract_bop_ycbv.py                # 기본 경로/설정
#   python3 03_extract_bop_ycbv.py --stride 3     # frame 간격 조절
#
# 입력:  ~/capstone_training/assets/bop_ycbv/  (ycbv_test_all.zip 압축 해제본)
# 출력:  ~/capstone_training/datasets/ycbv_yolo/{images,labels}/
#        파일명 = ycbv<scene>_<frame> → 06에서 scene 단위 split에 사용
#
# - 연속 frame은 거의 같은 장면이므로 --stride(기본 5)로 건너뛰며 추출
# - visib_fract(가려지지 않고 보이는 비율) < --min-visib 는 제외
# - bbox는 bbox_visib(보이는 영역 기준) 사용 — synthetic과 기준 일치
# =============================================================================
import argparse
import glob
import json
import os
import shutil
from collections import Counter

import cv2
import yaml

MIN_BOX_PX = 8


def parse_args():
    home = os.path.expanduser("~")
    p = argparse.ArgumentParser()
    p.add_argument("--bop-root",
                   default=os.path.join(home, "capstone_training", "assets", "bop_ycbv"))
    p.add_argument("--out",
                   default=os.path.join(home, "capstone_training", "datasets", "ycbv_yolo"))
    p.add_argument("--stride", type=int, default=5,
                   help="frame 샘플 간격 (연속 frame 중복 방지)")
    p.add_argument("--min-visib", type=float, default=0.3,
                   help="이 비율보다 덜 보이는(가려진) 물체는 제외")
    p.add_argument("--max-per-class", type=int, default=2000,
                   help="class당 이미지 수 상한")
    return p.parse_args()


def load_class_config():
    cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "config", "classes.yaml")
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    bop_map = {int(k): int(v) for k, v in cfg["bop_ycbv_obj_ids"].items()}
    names = {int(k): v for k, v in cfg["names"].items()}
    return bop_map, names


def find_rgb(scene_dir, frame_id):
    for sub, ext in (("rgb", "png"), ("rgb", "jpg")):
        path = os.path.join(scene_dir, sub, f"{frame_id:06d}.{ext}")
        if os.path.exists(path):
            return path
    return None


def main():
    args = parse_args()
    bop_map, names = load_class_config()

    scene_gt_paths = sorted(glob.glob(
        os.path.join(args.bop_root, "**", "scene_gt.json"), recursive=True))
    if not scene_gt_paths:
        raise SystemExit(
            f"scene_gt.json 없음: {args.bop_root}\n"
            "ycbv_test_all.zip 압축을 먼저 푸세요:\n"
            f"  unzip -q {args.bop_root}/ycbv_test_all.zip -d {args.bop_root}")

    os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "labels"), exist_ok=True)

    inst_count = Counter()      # class별 인스턴스(박스) 수
    img_count_per_class = Counter()
    n_saved = 0

    for sgt_path in scene_gt_paths:
        scene_dir = os.path.dirname(sgt_path)
        scene_id = os.path.basename(scene_dir)
        info_path = os.path.join(scene_dir, "scene_gt_info.json")
        if not os.path.exists(info_path):
            print(f"!! scene_gt_info.json 없음 → 건너뜀: {scene_dir}")
            continue
        with open(sgt_path) as f:
            gt = json.load(f)
        with open(info_path) as f:
            info = json.load(f)

        img_wh = None  # scene 내에서는 해상도 동일 — 첫 이미지에서 한 번만 읽음
        frame_ids = sorted((int(k) for k in gt.keys()))[::args.stride]

        for fid in frame_ids:
            fkey = str(fid)
            boxes = []  # (class_id, x, y, w, h)
            for g, gi in zip(gt.get(fkey, []), info.get(fkey, [])):
                cls = bop_map.get(int(g["obj_id"]))
                if cls is None:
                    continue
                if float(gi.get("visib_fract", 0.0)) < args.min_visib:
                    continue
                x, y, w, h = gi["bbox_visib"]
                if w < MIN_BOX_PX or h < MIN_BOX_PX:
                    continue
                boxes.append((cls, x, y, w, h))
            if not boxes:
                continue
            classes_here = {b[0] for b in boxes}
            if all(img_count_per_class[c] >= args.max_per_class for c in classes_here):
                continue

            rgb_path = find_rgb(scene_dir, fid)
            if rgb_path is None:
                continue
            if img_wh is None:
                im = cv2.imread(rgb_path)
                if im is None:
                    continue
                img_wh = (im.shape[1], im.shape[0])
            img_w, img_h = img_wh

            stem = f"ycbv{scene_id}_{fid:06d}"
            ext = os.path.splitext(rgb_path)[1]
            dst = os.path.join(args.out, "images", stem + ext)
            if not os.path.exists(dst):
                try:
                    os.link(rgb_path, dst)
                except OSError:
                    shutil.copy2(rgb_path, dst)
            lines = []
            for cls, x, y, w, h in boxes:
                xc = (x + w / 2.0) / img_w
                yc = (y + h / 2.0) / img_h
                lines.append(f"{cls} {xc:.6f} {yc:.6f} {w / img_w:.6f} {h / img_h:.6f}")
                inst_count[cls] += 1
            with open(os.path.join(args.out, "labels", stem + ".txt"), "w") as f:
                f.write("\n".join(lines) + "\n")
            for c in classes_here:
                img_count_per_class[c] += 1
            n_saved += 1

    print(f"\n== BOP ycbv 추출 완료: 이미지 {n_saved}장 ({len(scene_gt_paths)} scene 검사) ==")
    for cls in sorted(inst_count):
        print(f"  {names[cls]:<15} 인스턴스 {inst_count[cls]:>6} / 이미지 {img_count_per_class[cls]:>6}")
    if n_saved == 0:
        print("!! 추출 결과 0 — bop-root 경로와 압축 해제 상태를 확인하세요")
    print(f"출력: {args.out}")


if __name__ == "__main__":
    main()
