#!/usr/bin/env bash
# =============================================================================
# 01_download_assets.sh — 학습 데이터 원본 다운로드 (데스크톱에서 실행)
#
# 용법:
#   bash 01_download_assets.sh --meshes      # YCB CAD mesh 6종 (~수백 MB) ← 먼저
#   bash 01_download_assets.sh --coco        # COCO train2017 + annotations (~19 GB)
#   bash 01_download_assets.sh --bop-test    # BOP ycbv test_all (~15 GB, mustard/banana real)
#   bash 01_download_assets.sh --all         # 위 3개 전부
#   bash 01_download_assets.sh --coco-val    # (선택) COCO val2017 (~1 GB)
#   bash 01_download_assets.sh --bop-train   # (선택) ycbv train_real (대용량) — test_all로 부족할 때만
#
# - 이어받기 지원 (wget -c): 끊겨도 같은 명령 다시 실행하면 됨
# - 저장 위치: $HOME/capstone_training/assets  (ASSETS_DIR 환경변수로 변경 가능)
# - 사전 준비: sudo apt install -y wget curl unzip
# - URL은 2026-07-06 KV260 보드에서 HTTP 200 확인 완료
# =============================================================================
set -euo pipefail

ASSETS_DIR="${ASSETS_DIR:-$HOME/capstone_training/assets}"
YCB_BASE="http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data"
BOP_BASE="https://huggingface.co/datasets/bop-benchmark/ycbv/resolve/main"
COCO_IMG="http://images.cocodataset.org/zips"
COCO_ANN="http://images.cocodataset.org/annotations"

# classes.yaml의 ycb_objects와 동일 (6개 pickable)
YCB_OBJECTS=(013_apple 015_peach 017_orange 011_banana 056_tennis_ball 006_mustard_bottle)

do_meshes=false; do_coco=false; do_coco_val=false
do_bop_test=false; do_bop_train=false; do_bop_models=false

if [[ $# -eq 0 ]]; then
  sed -n '2,16p' "$0"
  exit 1
fi
for arg in "$@"; do
  case "$arg" in
    --meshes)     do_meshes=true ;;
    --coco)       do_coco=true ;;
    --coco-val)   do_coco_val=true ;;
    --bop-test)   do_bop_test=true ;;
    --bop-train)  do_bop_train=true ;;
    --bop-models) do_bop_models=true ;;
    --all)        do_meshes=true; do_coco=true; do_bop_test=true ;;
    *) echo "unknown flag: $arg" >&2; exit 1 ;;
  esac
done

fetch() { # fetch <url> <outdir>
  local url="$1" outdir="$2"
  mkdir -p "$outdir"
  echo ">> $url"
  wget -c -q --show-progress -P "$outdir" "$url"
}

if $do_meshes; then
  out="$ASSETS_DIR/ycb_meshes"
  for obj in "${YCB_OBJECTS[@]}"; do
    url="$YCB_BASE/google/${obj}_google_16k.tgz"
    if curl -sfIL "$url" >/dev/null 2>&1; then
      fetch "$url" "$out"
    else
      # 일부 물체는 google 스캔이 없어 berkeley mesh로 대체
      echo "!! google_16k 없음 → berkeley mesh 사용: $obj"
      fetch "$YCB_BASE/berkeley/${obj}/${obj}_berkeley_meshes.tgz" "$out"
    fi
  done
  echo ">> 압축 해제"
  for f in "$out"/*.tgz; do tar -xzf "$f" -C "$out"; done
  echo "OK: YCB meshes → $out"
fi

if $do_coco; then
  fetch "$COCO_IMG/train2017.zip" "$ASSETS_DIR/coco"
  fetch "$COCO_ANN/annotations_trainval2017.zip" "$ASSETS_DIR/coco"
  echo "NOTE: 압축 해제는 04 추출 스크립트 실행 전에:"
  echo "  unzip -q $ASSETS_DIR/coco/train2017.zip -d $ASSETS_DIR/coco"
  echo "  unzip -q $ASSETS_DIR/coco/annotations_trainval2017.zip -d $ASSETS_DIR/coco"
fi

if $do_coco_val; then
  fetch "$COCO_IMG/val2017.zip" "$ASSETS_DIR/coco"
fi

if $do_bop_models; then
  fetch "$BOP_BASE/ycbv_models.zip" "$ASSETS_DIR/bop_ycbv"
fi

if $do_bop_test; then
  fetch "$BOP_BASE/ycbv_test_all.zip" "$ASSETS_DIR/bop_ycbv"
  echo "NOTE: 압축 해제는 03 추출 스크립트 실행 전에:"
  echo "  unzip -q $ASSETS_DIR/bop_ycbv/ycbv_test_all.zip -d $ASSETS_DIR/bop_ycbv"
fi

if $do_bop_train; then
  echo "!! train_real은 대용량입니다. test_all 추출 결과(03)를 보고 부족할 때만 받으세요."
  fetch "$BOP_BASE/ycbv_train_real.zip" "$ASSETS_DIR/bop_ycbv"
fi

echo "완료. 저장 위치: $ASSETS_DIR"
