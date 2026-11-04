#!/usr/bin/env python3
# =============================================================================
# 12_quantize.py — vai_q_pytorch 양자화 (Vitis-AI 2.5 docker 안에서 실행)
#
# 직접 실행하지 말 것 — 12_run_quantize_docker.sh 가 docker 안에서 호출한다.
# (UG1414 v2.5 pp.86~107 흐름: calib → test → deploy/export_xmodel)
# --inspect 모드: 학습 전에 Inspector로 전 op의 DPU 배정을 예측 (Gate 0,
#   12a_inspect_docker.sh 가 호출) — SiLU 사고(Gate 3 시도1) 재발 방지.
#
# 구조 포인트:
# - yolov5의 Detect head는 grid 좌표 계산(sigmoid/grid 연산)이 섞여 있어
#   DPU에 못 올라간다. 그래서 DeployModel 래퍼가 backbone~neck까지만 돌리고
#   Detect 안의 마지막 1x1 conv 두 개를 직접 적용해 "raw conv 출력"
#   (bs,36,26,26)/(bs,36,13,13)만 반환한다. grid decode는 보드 worker가 수행.
# - decode에 필요한 anchors/strides/names는 decode_meta.json으로 함께 저장.
# - 전처리 = letterbox 416 (계획 D7) — 학습과 동일해야 calibration이 유효.
# =============================================================================
import argparse
import glob
import json
import os
import random
import sys

import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--weights", default=None, help="best.pt (container 경로)")
    p.add_argument("--quant_mode", choices=["calib", "test"], default=None)
    p.add_argument("--subset", type=int, default=500, help="calibration 이미지 수")
    p.add_argument("--deploy", action="store_true", help="(test 모드) xmodel export")
    p.add_argument("--inspect", action="store_true",
                   help="양자화 대신 Inspector로 DPU 매핑만 검사 (Gate 0)")
    p.add_argument("--cfg", default=None,
                   help="model yaml 경로 — 학습 전 구조만 검사할 때 사용 (무작위 가중치)")
    p.add_argument("--nc", type=int, default=7, help="--cfg 빌드 시 class 수")
    p.add_argument("--arch", default="/workspace/arch_b3136.json",
                   help="fingerprint를 읽을 arch json")
    p.add_argument("--img-dir", default="/workspace/datasets/final/images/train")
    p.add_argument("--out", default="/workspace/quantize")
    p.add_argument("--yolov5", default="/workspace/yolov5")
    p.add_argument("--img-size", type=int, default=416)
    return p.parse_args()


def letterbox_bgr(img, new=416, pad_value=114):
    """yolov5 val과 동일한 letterbox: 비율 유지 resize + 회색 padding."""
    import cv2
    h, w = img.shape[:2]
    r = min(new / h, new / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    if (nh, nw) != (h, w):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top = (new - nh) // 2
    bottom = new - nh - top
    left = (new - nw) // 2
    right = new - nw - left
    return cv2.copyMakeBorder(img, top, bottom, left, right,
                              cv2.BORDER_CONSTANT, value=(pad_value,) * 3)


def load_image_tensor(path, img_size):
    import cv2
    import torch
    img = cv2.imread(path)
    if img is None:
        return None
    img = letterbox_bgr(img, img_size)
    img = img[:, :, ::-1].copy()          # BGR → RGB (yolov5와 동일)
    t = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
    return t.unsqueeze(0)


def make_deploy(det_model):
    """DetectionModel → forward-only DeployModel + decode metadata."""
    import torch

    detect = det_model.model[-1]                # Detect head
    # DPU 미지원 activation(SiLU 등)이 섞여 들어오는 사고 방지용 가시화 (D11)
    print(f"[cfg] Conv activation = {type(det_model.model[0].act).__name__}")

    meta = {
        "names": (list(det_model.names.values())
                  if isinstance(det_model.names, dict) else list(det_model.names)),
        "strides": [int(s) for s in detect.stride.tolist()],
        # detect.anchors는 stride로 나눠진 값 → 픽셀 단위로 복원해 저장
        "anchors_pixel": [
            (detect.anchors[i] * detect.stride[i]).cpu().numpy().round(2).tolist()
            for i in range(detect.nl)
        ],
        "num_classes": int(detect.nc),
        "input_size": None,  # main에서 채움
        "letterbox_pad_value": 114,
        "output_order_note": "outputs[i]는 strides[i] scale의 raw conv (bs,3*(5+nc),H,W)",
    }

    class DeployModel(torch.nn.Module):
        def __init__(self, dm):
            super().__init__()
            self.layers = dm.model[:-1]     # Detect 제외 전체
            self.save = dm.save
            self.det_from = dm.model[-1].f  # Detect 입력 layer 인덱스들
            self.heads = dm.model[-1].m     # Detect 내부 1x1 conv들

        def forward(self, x):
            y = []
            for m in self.layers:
                if m.f != -1:
                    x = (y[m.f] if isinstance(m.f, int)
                         else [x if j == -1 else y[j] for j in m.f])
                x = m(x)
                y.append(x if m.i in self.save else None)
            feats = [y[j] for j in self.det_from]
            return tuple(head(f) for head, f in zip(self.heads, feats))

    return DeployModel(det_model).eval(), meta


def build_deploy_model(weights_path, yolov5_dir):
    """best.pt → DeployModel."""
    sys.path.insert(0, yolov5_dir)   # models/, utils/ import 가능하게
    import torch
    ckpt = torch.load(weights_path, map_location="cpu")
    return make_deploy(ckpt["model"].float().eval())  # fp16 저장분 → fp32


def build_from_cfg(cfg_path, yolov5_dir, nc):
    """model yaml → 무작위 가중치 DeployModel (학습 전 inspect 전용)."""
    sys.path.insert(0, yolov5_dir)
    from models.yolo import Model
    return make_deploy(Model(cfg_path, ch=3, nc=nc).float().eval())


def run_inspect(model, dummy, args):
    """Gate 0: Inspector로 모든 op의 DPU/CPU 배정 예측 (UG1414 v2.5 p.87)."""
    import torch
    try:
        from pytorch_nndct.apis import Inspector
    except ImportError:
        raise SystemExit("pytorch_nndct 없음 — Vitis-AI docker의 vitis-ai-pytorch "
                         "환경에서 실행해야 합니다 (12a_inspect_docker.sh 사용)")

    fingerprint = "0x101000016010406"   # DPUCZDX8G_ISA1_B3136 (kv260-smartcam)
    try:
        with open(args.arch) as f:
            fingerprint = json.load(f)["fingerprint"]
    except Exception:
        print(f"[inspect] {args.arch} 읽기 실패 — 기본 fingerprint 사용")
    print(f"[inspect] target fingerprint: {fingerprint}")

    inspector = Inspector(fingerprint)
    out_dir = os.path.join(args.out, "inspect")
    try:
        inspector.inspect(model, dummy, device=torch.device("cpu"),
                          output_dir=out_dir, image_format=None)
    except TypeError:
        # UG1414 v2.5 기본 시그니처 (리포트는 ./quantize_result 에 생성됨)
        inspector.inspect(model, dummy)

    reports = (glob.glob(os.path.join(out_dir, "inspect_*.txt"))
               + glob.glob("quantize_result/inspect_*.txt"))
    for rp in reports:
        print(f"\n[inspect] report: {rp}")
        cpu_lines = [ln.rstrip() for ln in open(rp, errors="ignore")
                     if "CPU" in ln]
        if cpu_lines:
            print(f"[inspect] 'CPU' 포함 줄 {len(cpu_lines)}개 — DPU 미배정 op 후보:")
            for ln in cpu_lines[:40]:
                print("   ", ln)
        else:
            print("[inspect] 리포트에 CPU 배정 없음 → 전 op DPU (Gate 0 통과)")
    if not reports:
        print("[inspect] 리포트 txt를 못 찾음 — 위 VAIQ 화면 출력으로 판정하세요")


def main():
    args = parse_args()
    import torch

    if args.inspect:
        if not (args.cfg or args.weights):
            raise SystemExit("--inspect 는 --cfg 또는 --weights 가 필요합니다")
    elif not (args.weights and args.quant_mode):
        raise SystemExit("--weights 와 --quant_mode 가 필요합니다 (또는 --inspect)")

    os.makedirs(args.out, exist_ok=True)
    if args.cfg:
        model, meta = build_from_cfg(args.cfg, args.yolov5, args.nc)
    else:
        model, meta = build_deploy_model(args.weights, args.yolov5)
    meta["input_size"] = args.img_size
    dummy = torch.randn(1, 3, args.img_size, args.img_size)

    if args.inspect:
        run_inspect(model, dummy, args)
        return

    try:
        from pytorch_nndct.apis import torch_quantizer
    except ImportError:
        raise SystemExit("pytorch_nndct 없음 — Vitis-AI docker의 vitis-ai-pytorch "
                         "환경에서 실행해야 합니다 (12_run_quantize_docker.sh 사용)")

    quantizer = torch_quantizer(args.quant_mode, model, (dummy,),
                                output_dir=os.path.join(args.out, "quantize_result"),
                                device=torch.device("cpu"))
    quant_model = quantizer.quant_model.eval()

    # ---- calibration / test forward ----
    paths = sorted(glob.glob(os.path.join(args.img_dir, "*")))
    random.seed(0)
    random.shuffle(paths)
    n = args.subset if not args.deploy else 1  # deploy는 batch=1 한 번이면 충분 (UG p.90)
    paths = paths[:n]
    print(f"[quant] mode={args.quant_mode} deploy={args.deploy} images={len(paths)}")

    cos_sums = None
    n_done = 0
    with torch.no_grad():
        for i, p in enumerate(paths):
            t = load_image_tensor(p, args.img_size)
            if t is None:
                continue
            q_out = quant_model(t)
            if args.quant_mode == "test" and not args.deploy:
                f_out = model(t)
                sims = [float(torch.nn.functional.cosine_similarity(
                    q.flatten(), f.flatten(), dim=0))
                    for q, f in zip(q_out, f_out)]
                cos_sums = sims if cos_sums is None else [a + b for a, b
                                                          in zip(cos_sums, sims)]
            n_done += 1
            if (i + 1) % 100 == 0:
                print(f"[quant] {i + 1}/{len(paths)}")

    if args.quant_mode == "calib":
        quantizer.export_quant_config()
        print("[quant] calibration 완료 → quant config 저장")
    elif args.deploy:
        quantizer.export_xmodel(
            output_dir=os.path.join(args.out, "quantize_result"),
            deploy_check=False)
        with open(os.path.join(args.out, "quantize_result",
                               "decode_meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        print("[quant] xmodel + decode_meta.json export 완료")
    else:
        sims = [s / max(1, n_done) for s in (cos_sums or [])]
        print("\n== float vs quantized 출력 유사도 (cosine, 1.0=동일) ==")
        for i, s in enumerate(sims):
            print(f"  head[{i}] (stride {meta['strides'][i]}): {s:.4f}")
        print("(0.99 이상이면 양자화 손실 매우 작음 / 0.95 미만이면 fast-finetune 검토)")


if __name__ == "__main__":
    main()
