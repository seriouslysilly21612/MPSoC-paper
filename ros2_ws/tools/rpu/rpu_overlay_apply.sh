#!/bin/bash
# RPU remoteproc overlay 적용/해제 — Gate 1 (docs/rpu/rpu_guide_for_claude.md §6)
# ⚠️ 08-03 미적용 초안. Gate 1 실제 적용본은 ~/rpu_work/rpu_v2.dtso(부팅 DTB 병합) — docs/rpu/rpu_guide_for_claude.md "정정 (2026-09-04)"
# 사용:  sudo bash rpu_overlay_apply.sh [apply|remove|status]
# runtime configfs 방식 — 재부팅하면 전부 원상복구된다. eth0/smartcam/CPU3 무관.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DTSO="$HERE/rpu_rproc.dtso"
DTBO="/tmp/rpu_rproc.dtbo"
OVL="/sys/kernel/config/device-tree/overlays/rpu"
# dtso의 reserved-memory와 반드시 일치 (0x3ed00000 ~ 0x3ed48000+0x100000)
RSV_START=$((0x3ed00000))
RSV_END=$((0x3ee48000))

status() {
    echo "== overlays =="; ls /sys/kernel/config/device-tree/overlays/ 2>/dev/null
    [ -d "$OVL" ] && echo "rpu overlay status: $(cat "$OVL/status" 2>/dev/null)"
    echo "== remoteproc =="; ls /sys/class/remoteproc/ 2>/dev/null || true
    for r in /sys/class/remoteproc/remoteproc*; do
        [ -d "$r" ] && echo "$r: $(cat "$r/state" 2>/dev/null) fw=$(cat "$r/firmware" 2>/dev/null)"
    done
    true
}

case "${1:-apply}" in
status)
    status; exit 0 ;;

remove)
    [ "$(id -u)" = 0 ] || { echo "sudo로 실행하세요"; exit 1; }
    for r in /sys/class/remoteproc/remoteproc*/state; do
        [ -f "$r" ] && grep -q running "$r" && echo stop > "$r" && echo "stopped $r"
    done
    rmdir "$OVL" && echo "overlay 제거됨 (모듈은 로드된 채 남음 — 무해)"
    exit 0 ;;

apply)
    [ "$(id -u)" = 0 ] || { echo "sudo로 실행하세요"; exit 1; }
    [ -d "$OVL" ] && { echo "이미 적용됨. 'status'로 확인, 재적용은 'remove' 후."; exit 1; }

    # 1) CMA 실배치와 충돌 검사 (reserved-memory는 runtime overlay라 커널이
    #    이미 쓰는 페이지일 수 있음 — 최소한 CMA와는 겹치지 않아야 함. §6-B 참조)
    CMA_LINE=$(dmesg 2>/dev/null | grep -m1 -iE 'cma:.*[Rr]eserved' || true)
    if [[ "$CMA_LINE" =~ at\ (0x[0-9a-fA-F]+) ]]; then
        CMA_BASE=$(( ${BASH_REMATCH[1]} ))
        CMA_END=$(( CMA_BASE + 1000*1024*1024 ))
        printf "CMA: base=0x%x end=0x%x\n" "$CMA_BASE" "$CMA_END"
        if [ "$RSV_START" -lt "$CMA_END" ] && [ "$CMA_BASE" -lt "$RSV_END" ]; then
            echo "!! RPU 예약영역(0x3ed00000~)이 CMA와 겹침 — dtso 주소를 옮기고 다시 실행"
            exit 1
        fi
    else
        echo "경고: dmesg에서 CMA 배치를 못 읽음(링버퍼 밀림). 'journalctl -k -b | grep -i cma'로 수동 확인 권장."
    fi
    grep -iE '3ed[0-9a-f]{5}|3ee[0-9a-f]{5}' /proc/iomem && { echo "!! iomem에 겹치는 항목 존재 — 위 출력 확인"; exit 1; } || true

    # 2) 컴파일 (문법/phandle 오류는 여기서 걸러짐)
    # -W no-*: overlay는 target 부모의 #address/#size-cells를 dtc가 볼 수 없어
    # reg/ranges 형식 경고가 나지만 런타임 병합 시(root·amba 모두 2/2) 올바르게
    # 해석된다 — 판정 기준은 exit code 0.
    dtc -@ -O dtb -o "$DTBO" "$DTSO" \
        -W no-reg_format -W no-ranges_format -W no-avoid_default_addr_size \
        -W no-unique_unit_address -W no-simple_bus_reg

    # 3) 적용 + probe
    mkdir "$OVL"
    cat "$DTBO" > "$OVL/dtbo"
    sleep 1
    echo "overlay status: $(cat "$OVL/status")"
    modprobe zynqmp_r5_remoteproc
    sleep 1
    dmesg | tail -15
    echo "----"
    if ls /sys/class/remoteproc/remoteproc* >/dev/null 2>&1; then
        status
        echo "OK: remoteproc 등장. 다음 → elf를 /lib/firmware/에 두고"
        echo "  echo <elf이름> > /sys/class/remoteproc/remoteproc0/firmware"
        echo "  echo start > /sys/class/remoteproc/remoteproc0/state"
    else
        echo "FAIL: remoteproc 미등장 — 위 dmesg에서 probe 오류 확인 (가이드 §6-C 진단 순서)"
        exit 1
    fi ;;
*)
    echo "사용법: sudo bash $0 [apply|remove|status]"; exit 1 ;;
esac
