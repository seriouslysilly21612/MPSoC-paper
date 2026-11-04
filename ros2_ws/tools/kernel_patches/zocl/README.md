# zocl KDS UAF 픽스 (2026-07-14)

## 무엇을 고치나
RT 커널에서 DPU 파이프라인 가동 시 발생한 **kmalloc-256 SLUB freelist 오염
→ 커널 Oops → 프리즈**(trace#2)의 근본 수정.

- **증거**: `~/ros2_ws/crash_logs/e2_poison_report_20260714-2336.log`
  (slub_debug=FZPU가 산 채로 잡은 리포트 — Allocated in `kds_alloc_command[zocl]`,
  Freed in `kds_free_command ← xrt_cu_intr_thread[zocl]`, 해제 후 offset 128에
  8바이트 `ktime` 값이 쓰임 = `timestamp[KDS_QUEUED]`)
- **원인 코드**: `drivers/gpu/drm/zocl/common/kds_core.c`의
  `xrt_cu_submit(...); set_xcmd_timestamp(xcmd, KDS_QUEUED);` — submit 후 스탬프.
  submit 순간부터 CU 스레드가 완료·해제 가능 → 해제된 객체에 스탬프 = UAF.
  offset 128은 프로덕션 SLUB(kmalloc-256, 하드닝)에서 freelist pointer 자리.
- RT(full preemption)가 레이스 창을 벌림. 순정에선 창이 거의 닫혀 있어 잠복.
- **upstream XRT master에도 잔존**(2026-07 확인) → XRT GitHub 리포트 권장.

## 적용 (PC, 커널 트리에서)
```bash
python3 apply_zocl_uaf_fix.py <tree>/drivers/gpu/drm/zocl/common/kds_core.c
# "[ok] 2~3건 교체 완료" + 검증 출력 확인
```

## 빌드 2가지 경로
**A. rev-6(-rt-kv260c)에 합류(권장)** — 패치 후 평소처럼 `bindeb-pkg` 전체 빌드.
rev-6 빌드가 이미 끝났으면 패치 적용 후 재빌드(증분이라 빠름).

**B. kv260b용 모듈만 교체(즉시 검증용)** — 같은 트리·같은 kv260b config에서:
```bash
make -j$(nproc) ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- M=drivers/gpu/drm/zocl modules
scp drivers/gpu/drm/zocl/zocl.ko ubuntu@192.168.120.132:/tmp/zocl.ko.fixed
```
보드에서 (sudo):
```bash
sudo cp /lib/modules/5.15.199-rt91-rt-kv260b/kernel/drivers/gpu/drm/zocl/zocl.ko{,.bak}
sudo cp /tmp/zocl.ko.fixed /lib/modules/5.15.199-rt91-rt-kv260b/kernel/drivers/gpu/drm/zocl/zocl.ko
sudo depmod -a 5.15.199-rt91-rt-kv260b && sudo reboot
```

## 검증 프로토콜 (패치 모듈로)
1. E2 계측 부팅 유지(`slub_debug=FZPU,kmalloc-256`) 상태에서 churn 루프 + 장시간 run
   → **Poison overwritten 0건**이면 픽스 확정.
2. 계측 제거(cmdline 원복 `skew_tick=1`) 후 프로덕션 소크.

## 별개 저심각 이슈 (trace#1, 선택)
`zocl_xclbin.c`의 `zocl_read_sect`가 `attr_rwlock`(RT에선 내부 rcu_read_lock 포함)
아래에서 vmalloc → sleeping-in-RCU 경고. 크래시와 무관(경고만).
수정 방향: Xilinx 자신이 같은 파일에서 `zocl_create_aie` 호출 전 쓰는
unlock→호출→relock 패턴을 `zocl_read_sect` 호출부에도 적용하거나,
섹션 버퍼를 락 진입 전에 미리 할당. rev-6 이후 여유 있을 때.
