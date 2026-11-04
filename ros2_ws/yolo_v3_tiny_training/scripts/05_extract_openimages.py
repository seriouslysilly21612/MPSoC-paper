#!/usr/bin/env python3
# =============================================================================
# 05_extract_openimages.py — Open Images → YOLO (peach / tennis_ball real 보조)
#
# peach와 tennis_ball은 COCO에 없어서 real 이미지가 부족한 class다.
# fiftyone의 zoo 다운로더로 Open Images에서 해당 class가 있는 이미지만
# 부분 다운로드해 YOLO로 변환한다.
#
# 용법 (데스크톱, venv):
#   python3 05_extract_openimages.py
#
# 출력: ~/capstone_training/datasets/openimages_yolo/{images,labels}/
#
# 주의:
# - 최초 실행 시 fiftyone이 Open Images metadata(수 GB)와 이미지들을
#   다운로드한다 → 첫 실행이 오래 걸림(수십 분). 재실행은 캐시 사용.
# - 같은 이미지에 있는 Apple/Orange/Banana/Person도 우리 class로 함께
#   라벨링한다 (unlabeled positive 방지).
# - IsGroupOf(뭉텅이 박스)가 우리 class에 있으면 그 이미지는 제외.
# =============================================================================
import argparse
import os
import shutil
from collections import Counter

# Open Images class 이름 → 우리 class id (classes.yaml과 일치해야 함)
OI_TO_OURS = {
    "Apple": 0,
    "Orange": 1,
    "Banana": 2,
    "Tennis ball": 3,
    "Person": 5,
}
TARGET_CLASSES = ["Tennis ball"]   # 다운로드 기준 class (D13: Peach 제거)
MIN_REL_SIZE = 0.008                        # 너무 작은 박스 제외 (~1024px 기준 8px)
NAMES = {0: "apple", 1: "orange", 2: "banana",
         3: "tennis_ball", 4: "mustard_bottle", 5: "person"}


def parse_args():
    home = os.path.expanduser("~")
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=os.path.join(
        home, "capstone_training", "datasets", "openimages_yolo"))
    p.add_argument("--split", default="train")
    p.add_argument("--max-each", type=int, default=1000,
                   help="class당 다운로드 이미지 수 (classes.yaml caps 참조)")
    return p.parse_args()


def get_flag(det, key):
    try:
        return bool(det[key])
    except Exception:
        return False


def main():
    args = parse_args()
    import fiftyone as fo
    import fiftyone.zoo as foz

    datasets = []
    for oi_cls in TARGET_CLASSES:
        ds_name = f"oi_{oi_cls.replace(' ', '_').lower()}_{args.split}_{args.max_each}"
        if fo.dataset_exists(ds_name):
            print(f"[oi] 캐시된 dataset 사용: {ds_name}")
            ds = fo.load_dataset(ds_name)
        else:
            print(f"[oi] '{oi_cls}' 다운로드 중... (최초 실행은 오래 걸림)")
            ds = foz.load_zoo_dataset(
                "open-images-v7",
                split=args.split,
                label_types=["detections"],
                classes=[oi_cls],
                max_samples=args.max_each,
                only_matching=False,   # 같은 이미지의 다른 class 라벨도 가져옴
                dataset_name=ds_name,
            )
            ds.persistent = True
        datasets.append(ds)

    os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "labels"), exist_ok=True)

    inst_count = Counter()
    seen = set()
    n_saved = 0
    n_group_skip = 0

    for ds in datasets:
        for sample in ds:
            fp = sample.filepath
            if fp in seen:
                continue
            seen.add(fp)

            field = getattr(sample, "ground_truth", None)
            dets = field.detections if field is not None else []
            boxes = []
            drop = False
            for det in dets:
                cls = OI_TO_OURS.get(det.label)
                if cls is None:
                    continue
                if get_flag(det, "IsGroupOf"):
                    drop = True  # 뭉텅이 박스 → 이미지 제외 (unlabeled positive 방지)
                    break
                x, y, w, h = det.bounding_box  # normalized [x, y, w, h]
                if w < MIN_REL_SIZE or h < MIN_REL_SIZE:
                    continue
                boxes.append((cls, x + w / 2.0, y + h / 2.0, w, h))
            if drop:
                n_group_skip += 1
                continue
            # 목적 class(peach/tennis_ball)가 하나도 없으면 저장하지 않음
            if not any(b[0] in (1, 4) for b in boxes):
                continue

            stem = "oi_" + os.path.splitext(os.path.basename(fp))[0]
            dst = os.path.join(args.out, "images",
                               stem + os.path.splitext(fp)[1])
            if not os.path.exists(dst):
                try:
                    os.link(fp, dst)
                except OSError:
                    shutil.copy2(fp, dst)
            with open(os.path.join(args.out, "labels", stem + ".txt"), "w") as f:
                for cls, xc, yc, w, h in boxes:
                    f.write(f"{cls} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")
                    inst_count[cls] += 1
            n_saved += 1

    print(f"\n== Open Images 추출 완료: 이미지 {n_saved}장 "
          f"(group-of 제외 {n_group_skip}장) ==")
    for cls in sorted(inst_count):
        print(f"  {NAMES[cls]:<15} 인스턴스 {inst_count[cls]:>6}")
    print(f"출력: {args.out}")


if __name__ == "__main__":
    main()
