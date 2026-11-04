#!/usr/bin/env python3
# =============================================================================
# 08_ycb_real_to_yolo.py — YCB 벤치마크 real 스캔(이미지+mask) → YOLO 학습 라벨
#
# YCB 물체 디렉토리(예: ~/006_mustard_bottle)는:
#   N<cam>_<angle>.jpg      (cam 1~5, angle 0~357, 총 600장; N4/N5가 top-down에 가까움)
#   masks/N<cam>_<angle>_mask.pbm  (1:1 대응, P4 이진 bitmap, object=소수 픽셀)
# 이 스크립트는 mask에서 tight bbox를 뽑아 YOLO txt(class cx cy w h, 정규화)로 저장하고
# 이미지를 out/images에 링크한다. (D12: real 데이터를 학습에 사용)
#
#   python3 08_ycb_real_to_yolo.py --ycb-dir ~/006_mustard_bottle --class-id 4 \
#       --cameras N4,N5 --stride 2 --out ~/ycb_real_yolo --viz 4
#
# --cameras : 사용할 카메라(콤마). 배포가 top-down이면 N4,N5 권장. 전부면 N1,N2,N3,N4,N5.
# --stride  : 각도 subsample(2=격각도). turntable 유사프레임 과다 방지.
# --viz     : bbox 오버레이 검증 이미지 N장 저장.
# =============================================================================
import argparse
import glob
import os
import re

import cv2
import numpy as np


def mask_bbox(mask_path):
    """P4 pbm mask → (x1,y1,x2,y2) in pixels, object=minority-value polarity."""
    m = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if m is None:
        return None
    vals, counts = np.unique(m, return_counts=True)
    if len(vals) < 2:
        return None
    obj_val = vals[np.argmin(counts)]          # 물체는 소수 픽셀(턴테이블 위 작은 물체)
    ys, xs = np.where(m == obj_val)
    if xs.size < 20:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()), m.shape[1], m.shape[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ycb-dir", required=True)
    ap.add_argument("--class-id", type=int, required=True)
    ap.add_argument("--cameras", default="N4,N5")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--out", required=True)
    ap.add_argument("--viz", type=int, default=0, help="bbox 검증 이미지 N장")
    ap.add_argument("--min-frac", type=float, default=0.0005,
                    help="bbox 면적이 프레임의 이 비율 미만이면 스킵(노이즈)")
    ap.add_argument("--max-frac", type=float, default=0.9,
                    help="이 비율 초과면 스킵(mask polarity 오류 방지)")
    args = ap.parse_args()

    cams = set(args.cameras.split(","))
    img_out = os.path.join(args.out, "images")
    lbl_out = os.path.join(args.out, "labels")
    viz_out = os.path.join(args.out, "viz")
    for d in (img_out, lbl_out, viz_out):
        os.makedirs(d, exist_ok=True)

    stem = os.path.basename(os.path.abspath(args.ycb_dir))   # e.g. 006_mustard_bottle
    jpgs = sorted(glob.glob(os.path.join(args.ycb_dir, "N*_*.jpg")))
    n_ok = n_skip = n_viz = 0
    for jp in jpgs:
        name = os.path.splitext(os.path.basename(jp))[0]     # N5_０
        mobj = re.match(r"(N\d)_(\d+)$", name)
        if not mobj:
            continue
        cam, ang = mobj.group(1), int(mobj.group(2))
        if cam not in cams or (ang // 3) % args.stride != 0:  # 각도는 3도 간격
            continue
        mask_path = os.path.join(args.ycb_dir, "masks", f"{name}_mask.pbm")
        bb = mask_bbox(mask_path)
        if bb is None:
            n_skip += 1
            continue
        x1, y1, x2, y2, W, H = bb
        frac = ((x2 - x1) * (y2 - y1)) / float(W * H)
        if frac < args.min_frac or frac > args.max_frac:
            n_skip += 1
            continue
        cx = (x1 + x2) / 2.0 / W
        cy = (y1 + y2) / 2.0 / H
        bw = (x2 - x1) / float(W)
        bh = (y2 - y1) / float(H)
        out_stem = f"ycb_{stem}_{name}"
        with open(os.path.join(lbl_out, out_stem + ".txt"), "w") as f:
            f.write(f"{args.class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
        link = os.path.join(img_out, out_stem + ".jpg")
        if not os.path.exists(link):
            try:
                os.link(jp, link)
            except OSError:
                import shutil
                shutil.copy(jp, link)
        n_ok += 1
        if n_viz < args.viz:
            img = cv2.imread(jp)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 6)
            cv2.imwrite(os.path.join(viz_out, out_stem + "_viz.jpg"), img)
            n_viz += 1

    print(f"[ycb->yolo] {stem} class={args.class_id} cams={sorted(cams)} "
          f"stride={args.stride}: wrote {n_ok}, skipped {n_skip}, viz {n_viz}")
    print(f"  images: {img_out}\n  labels: {lbl_out}\n  viz: {viz_out}")


if __name__ == "__main__":
    main()
