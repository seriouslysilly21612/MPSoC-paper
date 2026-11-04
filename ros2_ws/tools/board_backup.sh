#!/bin/bash
# board_backup.sh — SD 사망 대비, git에 없는 보드 로컬 상태 백업 (2026-08-03)
#
# git+apt 복구 계획(우분투 재플래시 → RT커널 deb → ROS2 → repo clone)에서
# 빠지는 것들만 tar 하나로 묶는다. 목록·근거: docs/RAON-RT/merge.md.
#
#   사용:  sudo bash ~/ros2_ws/tools/board_backup.sh
#          (sudo 없이도 동작하지만 /etc/sudoers.d 등 root 전용 파일은 빠짐)
#   보관:  scp ~/board_backup_*.tar.gz <PC>:
#   주기:  실기 세션 끝날 때마다 — INDY7.cfg는 앱이 종료 시 엔코더 카운트를
#          써넣는 라이브 파일이라 오래된 백업은 E20/E24류 위험이 있다.
#   복원:  tar 안의 state/RESTORE.md 참조 (요지: repo clone을 먼저 하고
#          그 다음 sudo tar -xzf backup.tar.gz -C / 로 제자리 복원)

set -u
H=/home/ubuntu
OUT="$H/board_backup_$(date +%Y%m%d_%H%M%S).tar.gz"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# ---- 스냅샷 상태 파일 (state/) --------------------------------------------
mkdir -p "$TMP/state"
cat /proc/cmdline > "$TMP/state/cmdline.txt"
uname -r          > "$TMP/state/kernel.txt"
{
  echo "RAON-RT-Revision: $(git -C $H/RAON-RT-Revision rev-parse --short HEAD 2>/dev/null) ($(git -C $H/RAON-RT-Revision branch --show-current 2>/dev/null))"
  echo "ros2_ws:          $(git -C $H/ros2_ws rev-parse --short HEAD 2>/dev/null) ($(git -C $H/ros2_ws branch --show-current 2>/dev/null))"
} > "$TMP/state/git_heads.txt"

cat > "$TMP/state/RESTORE.md" <<'EOF'
# 복원 절차 (요지 — 상세 근거는 ros2_ws docs/RAON-RT/merge.md)

1. Kria Ubuntu 22.04 플래시, 사용자 ubuntu 생성.
2. RT 커널 deb 설치 (linux-image/headers 5.15.199-rt91-rt-kv260c **리비전 -11**) 후 재부팅.
   ⚠️ 반드시 -11 — -10과 커널 버전 문자열이 같아 헷갈리지만, -11이 07-20 ASIX
   재빌드(usbnet/mii =m)라서 -10으로 깔면 AX88179B NIC(EtherCAT 경로)이 안 산다.
   deb 보관처: ros2_ws repo의 recovery/ (2026-08-06 커밋) + 보드 ~/ + 크로스컴파일 PC.
   백업 tar에는 크기(70MB) 때문에 안 들어간다.
3. 저장소 clone (경로 고정 — tar가 이 경로 안으로 파일을 덮어씀):
   git clone <RAON-RT-Revision> ~/RAON-RT-Revision  (branch kv260-merge)
   git clone <Capstone_Design>  ~/ros2_ws
4. sudo tar -xzf board_backup_*.tar.gz -C /
   → INDY7.cfg(라이브), ready seed, /opt 3종(etherlab/rt_posix/emaster_app),
     IgH·ASIX 커널모듈, udev/modprobe/flash-kernel 설정, 가이드 문서, Claude 메모리 복원.
5. sudo depmod -a && sudo flash-kernel   # 모듈 인덱스 + cmdline(image.fit) 재생성 → 재부팅
   재부팅 후 cat /proc/cmdline 이 state/cmdline.txt 와 같은지 확인.
6. ROS2 Humble + 의존 apt 설치(ros2_ws CLAUDE.md 참조) → cd ~/ros2_ws && colcon build.
   DPU 스택은 전부 apt 재설치 가능(2026-08-05 확인 — tar가 /etc/apt/sources.list.d 의
   Xilinx PPA 목록을 복원함): sudo apt install vitis-ai-runtime xlnx-firmware-kv260-smartcam
   realsense는 ros-humble-librealsense2 / ros-humble-realsense2-camera (ROS apt).
   DPU 부팅 자동로드: tar가 복원한 유닛을 sudo systemctl enable kv260-smartcam.service
7. RAON 앱 빌드: make -C ~/RAON-RT-Revision/App/Indy7   ("make clean" 금지 — merge.md 수칙)
   sudo bash ~/RAON-RT-Revision/tools/install_ecat_op_fifo.sh   # E34 chrt 1회 설치
8. 검증: IgH 기동(ethercatctl/서비스, igh 메모리 참조) → run.sh → 'r,g' 스모크(servo-off)
   → 첫 실기 전 인코더/HOME 상태 재확인 (E20/E24: 카운터 기준은 세션마다 달라질 수 있음).

주의: ~/.indy7_ready_seed 를 잃고 HOME을 재기록하면 reach map 검증(99.4%)이 무효가 된다.
이 tar의 seed를 쓰는 한 재기록 금지.
EOF

# ---- 백업 대상 (모두 / 기준 상대 경로; 없으면 경고만) ----------------------
KREL=$(uname -r)
ITEMS=(
  # 1) 라이브 오퍼레이터 상태 (git에 절대 커밋 안 함)
  "home/ubuntu/RAON-RT-Revision/App/Indy7/INDY7.cfg"
  "home/ubuntu/.indy7_ready_seed"
  # 2) $HOME 로컬 전용 스크립트·문서·Claude 메모리
  "home/ubuntu/setup_ax_usb_nic.sh"
  "home/ubuntu/RAON-RT_guide.md"
  "home/ubuntu/RAON-RT_guide_for_CLAUDE.md"
  "home/ubuntu/RAON-RT_study.md"
  "home/ubuntu/ASIX_USB_NIC_Linux_Driver_Source_v4.1.0.tar.bz2"
  "home/ubuntu/.claude/projects/-home-ubuntu/memory"
  "home/ubuntu/.claude/plans"
  # 3) /opt 수동 빌드 스택 (없으면 Indy7Ctrl.out 빌드 불가)
  "opt/etherlab"
  "opt/rt_posix"
  "opt/emaster_app"
  # 4) 커널 모듈 (RT 커널 버전 전용 — deb 재설치 후 그대로 유효)
  "lib/modules/$KREL/ethercat"
  "lib/modules/$KREL/kernel/drivers/net/usb/ax_usb_nic.ko"
  # 5) 시스템 설정
  "etc/default/flash-kernel"
  "etc/modprobe.d/ax_usb_nic_blacklist.conf"
  "etc/udev/rules.d"
  "etc/sudoers.d/ecat-op-fifo"
  "lib/systemd/system/ethercat.service"
  "usr/local/sbin/ecat-op-fifo"
  # 6) DPU 부팅 자동로드 수제 유닛 + Xilinx PPA 소스 목록 (2026-08-05 점검에서
  #    누락 발견 — 유닛은 수제 274B라 재플래시 이미지에 없고, PPA 목록은 재설치
  #    가능성의 전제. vitis-ai-runtime 등 패키지 자체는 apt로 복구된다.)
  "etc/systemd/system/kv260-smartcam.service"
  "etc/apt/sources.list.d"
)

EXIST=()
for p in "${ITEMS[@]}"; do
  if [ -e "/$p" ]; then EXIST+=("$p"); else echo "경고: /$p 없음 — 건너뜀"; fi
done
[ "$(id -u)" -ne 0 ] && echo "경고: sudo 아님 — root 전용 파일(sudoers 등)은 빠질 수 있음"

tar -czf "$OUT" --ignore-failed-read -C / "${EXIST[@]}" -C "$TMP" state
[ -n "${SUDO_USER:-}" ] && chown "$SUDO_USER:$SUDO_USER" "$OUT"

# 보드에는 최신 3개만 유지
ls -t "$H"/board_backup_*.tar.gz 2>/dev/null | tail -n +4 | xargs -r rm

echo
echo "생성: $OUT ($(du -h "$OUT" | cut -f1))"
echo "PC 보관 (원격 파일명까지 명시 — 일부 scp는 'dir/: Is a directory'로 실패):"
echo "  scp $OUT <user>@<pc>:kv260_backups/$(basename "$OUT")"
