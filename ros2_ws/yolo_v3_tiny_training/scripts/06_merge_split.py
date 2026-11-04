#!/usr/bin/env python3
# =============================================================================
# 06_merge_split.py — 소스별 YOLO 데이터 병합 + train/val split
#
# 입력(존재하는 것만 자동 사용):
#   datasets/synth_yolo/       (02b 출력)
#   datasets/ycbv_yolo/        (03 출력)
#   datasets/coco_yolo/        (04 출력)
#   datasets/openimages_yolo/  (05 출력)
# 출력:
#   datasets/final/images/{train,val}/  + labels/{train,val}/  + data.yaml
#
# split 규칙 (같은 장면이 train/val 양쪽에 가지 않도록):
#   - synth  : batch 단위로 val 배정 (한 batch 안의 scene들이 통째로 이동)
#   - ycbv   : scene 단위로 val 배정 (연속 video frame이 갈라지지 않게)
#   - coco/oi: 독립 사진이므로 이미지 단위 random 배정
#
# 용법 (데스크톱, venv):
#   python3 06_merge_split.py              # 기본 val 비율 10%
#   python3 06_merge_split.py --val-frac 0.15
# =============================================================================
import argparse
import math
import os
import random
import shutil
from collections import Counter, defaultdict

# 6-class (D13: peach 드롭). classes.yaml과 동일 순서.
NAMES = ["apple", "orange", "banana", "tennis_ball", "mustard_bottle", "person"]
SOURCES = ["synth_yolo", "ycbv_yolo", "coco_yolo", "openimages_yolo", "ycb_real_yolo",
           "real_apple_yolo"]
# YCB 벤치마크 real 스캔(턴테이블) + 배포 카메라 실촬영 apple → 학습 증강용 전량 train
# (val은 실장면 board 검증으로 대체; 실촬영을 val에 넣으면 같은 station 과대평가됨)
# real_apple_yolo: apple color 도메인 갭 해소용 실물 top-down(단일물체 auto-label), 2026-07-09
TRAIN_ONLY_SOURCES = {"ycb_real_yolo", "real_apple_yolo"}


def parse_args():
    home = os.path.expanduser("~")
    p = argparse.ArgumentParser()
    p.add_argument("--root",
                   default=os.path.join(home, "capstone_training", "datasets"))
    p.add_argument("--out", default=None, help="기본: <root>/final")
    p.add_argument("--val-frac", type=float, default=0.10)
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args()


def group_key(stem, source):
    """같은 '장면'에 속하는 이미지들이 같은 key를 갖게 한다."""
    if source == "synth_yolo":          # batch_003_000012 → batch_003
        parts = stem.split("_")
        return "_".join(parts[:2]) if len(parts) >= 2 else stem
    if source == "ycbv_yolo":           # ycbv000048_000123 → ycbv000048
        return stem.split("_")[0]
    return stem                          # coco/oi: 이미지 단위


def link_or_copy(src, dst):
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def main():
    args = parse_args()
    random.seed(args.seed)
    out = args.out or os.path.join(args.root, "final")

    # 수집: (source, stem, img_path, lbl_path, group)
    items = []
    for source in SOURCES:
        img_dir = os.path.join(args.root, source, "images")
        lbl_dir = os.path.join(args.root, source, "labels")
        if not os.path.isdir(img_dir):
            print(f"[merge] {source} 없음 → 건너뜀")
            continue
        n = 0
        for fname in os.listdir(img_dir):
            stem, ext = os.path.splitext(fname)
            lbl = os.path.join(lbl_dir, stem + ".txt")
            if not os.path.exists(lbl):
                continue
            items.append((source, stem, os.path.join(img_dir, fname), lbl,
                          group_key(stem, source)))
            n += 1
        print(f"[merge] {source}: {n}장")
    if not items:
        raise SystemExit("병합할 데이터가 없습니다 — 02b/03/04/05를 먼저 실행하세요")

    # 재실행 시 이전 split 잔재가 train/val 중복을 만들지 않도록 항상 새로 생성
    if os.path.isdir(out):
        print(f"[merge] 기존 출력 삭제 후 재생성: {out}")
        shutil.rmtree(out)

    # split: group 단위
    val_groups = set()
    by_source_groups = defaultdict(set)
    for source, _, _, _, grp in items:
        by_source_groups[source].add(grp)
    for source, groups in by_source_groups.items():
        if source in TRAIN_ONLY_SOURCES:      # ycb_real → 전량 train
            continue
        groups = sorted(groups)
        random.shuffle(groups)
        n_val = max(1, math.ceil(len(groups) * args.val_frac))
        val_groups.update((source, g) for g in groups[:n_val])

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(out, sub), exist_ok=True)

    img_counts = Counter()
    inst_counts = {"train": Counter(), "val": Counter()}
    for source, stem, img_path, lbl_path, grp in items:
        split = "val" if (source, grp) in val_groups else "train"
        ext = os.path.splitext(img_path)[1]
        dst_img = os.path.join(out, "images", split, stem + ext)
        dst_lbl = os.path.join(out, "labels", split, stem + ".txt")
        if not os.path.exists(dst_img):
            link_or_copy(img_path, dst_img)
        if not os.path.exists(dst_lbl):
            link_or_copy(lbl_path, dst_lbl)
        img_counts[(source, split)] += 1
        with open(lbl_path) as f:
            for line in f:
                line = line.strip()
                if line:
                    inst_counts[split][int(line.split()[0])] += 1

    # data.yaml (ultralytics/yolov3 형식)
    with open(os.path.join(out, "data.yaml"), "w") as f:
        f.write(f"train: {os.path.join(out, 'images', 'train')}\n")
        f.write(f"val: {os.path.join(out, 'images', 'val')}\n")
        f.write(f"nc: {len(NAMES)}\n")
        f.write("names: [" + ", ".join(NAMES) + "]\n")

    print("\n== 병합/split 결과 ==")
    for source in SOURCES:
        tr = img_counts.get((source, "train"), 0)
        va = img_counts.get((source, "val"), 0)
        if tr or va:
            print(f"  {source:<17} train {tr:>6} / val {va:>5}")
    total_tr = sum(v for (s, sp), v in img_counts.items() if sp == "train")
    total_va = sum(v for (s, sp), v in img_counts.items() if sp == "val")
    print(f"  {'TOTAL':<17} train {total_tr:>6} / val {total_va:>5}")
    print("\n== class별 인스턴스 (train / val) ==")
    for cls, name in enumerate(NAMES):
        print(f"  {name:<15} {inst_counts['train'][cls]:>7} / {inst_counts['val'][cls]:>6}")
    print(f"\n출력: {out}  (data.yaml 포함)")


if __name__ == "__main__":
    main()
