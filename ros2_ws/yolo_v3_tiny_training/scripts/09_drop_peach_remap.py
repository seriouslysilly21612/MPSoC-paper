#!/usr/bin/env python3
# =============================================================================
# 09_drop_peach_remap.py — 기존 7-class YOLO 라벨을 6-class로 1회 remap (D13)
#
# peach(id 1) 드롭 → 재번호. 기존 추출물(02b/03/04/05 출력)을 재추출 없이 변환.
#   old→new: {0:0, 2:1, 3:2, 4:3, 5:4, 6:5},  peach(1)=삭제
#   apple 0→0 / orange 2→1 / banana 3→2 / tennis 4→3 / mustard 5→4 / person 6→5
# peach만 있던 이미지(라벨이 비게 됨)는 image+label 삭제(비라벨 positive 방지).
#
# ⚠️ 1회성. classes.yaml/스크립트가 이미 6-class로 바뀐 뒤, "기존 7-class 데이터"에만
#    실행. 6-class 데이터에 다시 돌리면 orange를 peach로 오인해 손상됨 → 안전가드 있음.
#
#   python3 09_drop_peach_remap.py                 # ~/capstone_training/datasets 전체
#   python3 09_drop_peach_remap.py --root <dir> --dry-run
# =============================================================================
import argparse
import glob
import os

REMAP = {0: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5}   # peach(1) 제외
DROP = 1
SOURCES = ["synth_yolo", "ycbv_yolo", "coco_yolo", "openimages_yolo"]


def label_dirs(root):
    # source dir만 remap한다. final/은 06_merge_split.py가 source에서 재생성하므로
    # 건드리지 않는다 (06이 source→final을 hardlink로 연결 → final까지 처리하면
    # 같은 inode를 두 번 remap해 손상됨).
    dirs = []
    for s in SOURCES:
        d = os.path.join(root, s, "labels")
        if os.path.isdir(d):
            dirs.append((s, d))
    return dirs


def find_image(lbl_path):
    d = lbl_path.replace(os.sep + "labels" + os.sep, os.sep + "images" + os.sep)
    stem = os.path.splitext(d)[0]
    for ext in (".jpg", ".jpeg", ".png", ".bmp"):
        if os.path.exists(stem + ext):
            return stem + ext
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.expanduser("~/capstone_training/datasets"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force", action="store_true", help="안전가드 무시(권장 안 함)")
    args = ap.parse_args()

    dirs = label_dirs(args.root)
    if not dirs:
        raise SystemExit(f"라벨 디렉토리 없음: {args.root}")

    # 안전가드: 전체에서 class 6(구 person)이 하나라도 있어야 7-class로 판단
    max_cls = -1
    for _, d in dirs:
        for lf in glob.glob(os.path.join(d, "*.txt")):
            with open(lf) as f:
                for line in f:
                    p = line.split()
                    if p:
                        max_cls = max(max_cls, int(p[0]))
            if max_cls >= 6:
                break
        if max_cls >= 6:
            break
    if max_cls < 6 and not args.force:
        raise SystemExit(f"class 6(구 person) 미발견 → 이미 6-class로 remap된 데이터로 "
                         f"보임(max class={max_cls}). 재실행 방지. 강제하려면 --force")

    print(f"[remap] root={args.root}  dirs={len(dirs)}  {'(dry-run)' if args.dry_run else ''}")
    seen_inodes = set()          # hardlink 이중 remap 방지 (같은 inode 1회만)
    for src, d in dirs:
        n_files = n_dropped = n_emptied = n_lines = 0
        for lf in glob.glob(os.path.join(d, "*.txt")):
            ino = os.stat(lf).st_ino
            if ino in seen_inodes:
                continue
            seen_inodes.add(ino)
            n_files += 1
            out_lines = []
            with open(lf) as f:
                for line in f:
                    parts = line.split()
                    if not parts:
                        continue
                    c = int(parts[0])
                    if c == DROP:
                        n_dropped += 1
                        continue
                    if c not in REMAP:
                        continue
                    parts[0] = str(REMAP[c])
                    out_lines.append(" ".join(parts))
                    n_lines += 1
            if out_lines:
                if not args.dry_run:
                    with open(lf, "w") as f:
                        f.write("\n".join(out_lines) + "\n")
            else:
                # peach-only → image+label 삭제
                n_emptied += 1
                if not args.dry_run:
                    img = find_image(lf)
                    os.remove(lf)
                    if img:
                        os.remove(img)
        print(f"  {src:<15} files {n_files:>6} | peach줄삭제 {n_dropped:>6} | "
              f"빈이미지삭제 {n_emptied:>5} | 남은라벨 {n_lines:>7}")
    print("완료. 다음: 08(YCB 변환) → 06_merge_split.py 재실행")


if __name__ == "__main__":
    main()
