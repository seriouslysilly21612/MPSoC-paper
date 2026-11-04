#!/usr/bin/env python3
# =============================================================================
# 07_contactsheet.py — annotation 육안 검수용 contact-sheet 생성
#
# class마다 그 class가 들어있는 이미지를 랜덤 샘플해 bbox를 그려 넣고
# 한 장의 큰 격자 이미지(contact sheet)로 만든다. 사람 눈으로
# "박스가 물체에 제대로 붙어 있는가"를 빠르게 확인하는 용도.
#
# 용법 (데스크톱, venv, 06 실행 후):
#   python3 07_contactsheet.py
#   → datasets/contact_sheets/<class>.jpg 7장 + negatives.jpg
# =============================================================================
import argparse
import os
import random
from collections import defaultdict

import cv2
import numpy as np

NAMES = ["apple", "orange", "banana",
         "tennis_ball", "mustard_bottle", "person"]
COLORS = [(60, 76, 231), (18, 156, 243), (96, 174, 39),
          (196, 229, 56), (94, 73, 52), (185, 128, 41)]  # BGR (6-class)


def parse_args():
    home = os.path.expanduser("~")
    p = argparse.ArgumentParser()
    p.add_argument("--data",
                   default=os.path.join(home, "capstone_training", "datasets", "final"),
                   help="06 출력 디렉토리 (images/labels 하위 포함)")
    p.add_argument("--out",
                   default=os.path.join(home, "capstone_training", "datasets",
                                        "contact_sheets"))
    p.add_argument("--per-class", type=int, default=40)
    p.add_argument("--cols", type=int, default=8)
    p.add_argument("--thumb-width", type=int, default=360)
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args()


def collect(data_dir):
    """split 구분 없이 (img, lbl) 쌍 수집."""
    pairs = []
    for split in ("train", "val"):
        img_dir = os.path.join(data_dir, "images", split)
        lbl_dir = os.path.join(data_dir, "labels", split)
        if not os.path.isdir(img_dir):
            continue
        for fname in os.listdir(img_dir):
            stem = os.path.splitext(fname)[0]
            lbl = os.path.join(lbl_dir, stem + ".txt")
            if os.path.exists(lbl):
                pairs.append((os.path.join(img_dir, fname), lbl))
    return pairs


def draw_boxes(img, lbl_path):
    h, w = img.shape[:2]
    with open(lbl_path) as f:
        for line in f:
            parts = line.split()
            if len(parts) != 5:
                continue
            cls = int(parts[0])
            xc, yc, bw, bh = (float(v) for v in parts[1:])
            x1 = int((xc - bw / 2) * w); y1 = int((yc - bh / 2) * h)
            x2 = int((xc + bw / 2) * w); y2 = int((yc + bh / 2) * h)
            color = COLORS[cls % len(COLORS)]
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, NAMES[cls], (x1, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return img


def make_sheet(samples, thumb_w, cols):
    thumbs = []
    for img_path, lbl_path in samples:
        img = cv2.imread(img_path)
        if img is None:
            continue
        img = draw_boxes(img, lbl_path)
        scale = thumb_w / img.shape[1]
        img = cv2.resize(img, (thumb_w, int(img.shape[0] * scale)))
        thumbs.append(img)
    if not thumbs:
        return None
    max_h = max(t.shape[0] for t in thumbs)
    thumbs = [cv2.copyMakeBorder(t, 0, max_h - t.shape[0], 0, 0,
                                 cv2.BORDER_CONSTANT, value=(30, 30, 30))
              for t in thumbs]
    rows = []
    for i in range(0, len(thumbs), cols):
        row = thumbs[i:i + cols]
        while len(row) < cols:
            row.append(np.full_like(thumbs[0], 30))
        rows.append(np.hstack(row))
    return np.vstack(rows)


def main():
    args = parse_args()
    random.seed(args.seed)
    pairs = collect(args.data)
    if not pairs:
        raise SystemExit(f"images/labels 없음: {args.data} — 06을 먼저 실행하세요")
    os.makedirs(args.out, exist_ok=True)

    by_class = defaultdict(list)
    negatives = []
    for img_path, lbl_path in pairs:
        with open(lbl_path) as f:
            classes = {int(l.split()[0]) for l in f if l.strip()}
        if not classes:
            negatives.append((img_path, lbl_path))
        for c in classes:
            by_class[c].append((img_path, lbl_path))

    for cls, name in enumerate(NAMES):
        pool = by_class.get(cls, [])
        if not pool:
            print(f"!! {name}: 이미지 0장 — 데이터 소스 확인 필요")
            continue
        samples = random.sample(pool, min(args.per_class, len(pool)))
        sheet = make_sheet(samples, args.thumb_width, args.cols)
        if sheet is None:
            continue
        out_path = os.path.join(args.out, f"{cls}_{name}.jpg")
        cv2.imwrite(out_path, sheet, [cv2.IMWRITE_JPEG_QUALITY, 82])
        print(f"[sheet] {name:<15} {len(pool):>6}장 중 {len(samples)}장 → {out_path}")

    if negatives:
        samples = random.sample(negatives, min(args.per_class, len(negatives)))
        sheet = make_sheet(samples, args.thumb_width, args.cols)
        if sheet is not None:
            out_path = os.path.join(args.out, "negatives.jpg")
            cv2.imwrite(out_path, sheet, [cv2.IMWRITE_JPEG_QUALITY, 82])
            print(f"[sheet] negatives       {len(negatives):>6}장 중 {len(samples)}장 → {out_path}")

    print(f"\n검수 방법: {args.out} 의 jpg들을 열어 박스가 물체에 정확히 붙어있는지,"
          "\n엉뚱한 라벨이 없는지 눈으로 확인하세요.")


if __name__ == "__main__":
    main()
