#!/usr/bin/env python3
"""zocl KDS use-after-free fix (2026-07-14, E2 slub_debug로 규명).

버그: kds_cu_dispatch/kds_scu_dispatch/kds_submit_ert가 command를
xrt_cu_submit()/ert->submit()으로 CU·ERT 스레드에 넘긴 *다음에*
set_xcmd_timestamp(xcmd, KDS_QUEUED)를 호출한다. submit 순간부터 CU
인터럽트 스레드가 command를 완료→kds_free_command로 해제할 수 있어,
제출 스레드가 freed 객체의 timestamp[KDS_QUEUED](offset 128 =
프로덕션 SLUB freelist pointer 자리)에 ktime을 쓰는 UAF가 된다.
RT 커널(full preemption)이 이 창을 벌려 kmalloc-256 freelist 오염
→ ___slab_alloc Oops → 시스템 프리즈로 발현됐다 (upstream XRT
master에도 동일 버그 존재, 2026-07 기준 미수정).

수정: 스탬프를 submit *앞*으로 이동 (의미 동일 — "큐에 넣은 시각").

사용법 (PC 커널 트리에서):
  python3 apply_zocl_uaf_fix.py <tree>/drivers/gpu/drm/zocl/common/kds_core.c
교체 건수 2~3건이 정상(cu/scu/ert dispatch). 0건이면 트리 버전 확인.
원본은 kds_core.c.orig로 백업된다.
"""
import re
import shutil
import sys

path = sys.argv[1]
src = open(path).read()

pat = re.compile(
    r"(?P<submit>[ \t]*(?:xrt_cu_submit\([^;\n]*\)|ert->submit\(ert, xcmd\));[ \t]*\n)"
    r"(?P<stamp>[ \t]*set_xcmd_timestamp\(xcmd, KDS_QUEUED\);[ \t]*\n)"
)
new, n = pat.subn(lambda m: m.group("stamp") + m.group("submit"), src)

if n == 0:
    print("[!] 대상 패턴 0건 — 이미 패치됐거나 트리 버전이 다름. 수동 확인 필요:")
    print("    grep -n -B2 'set_xcmd_timestamp(xcmd, KDS_QUEUED)' " + path)
    sys.exit(1)

shutil.copy2(path, path + ".orig")
open(path, "w").write(new)
print(f"[ok] {n}건 교체 완료 (백업: {path}.orig)")
print("[검증] 아래 출력에서 각 set_xcmd_timestamp(KDS_QUEUED)가 submit *앞*줄인지 확인:")
for i, line in enumerate(new.splitlines(), 1):
    if "KDS_QUEUED" in line and "set_xcmd_timestamp" in line:
        print(f"  {i}: {line.strip()}")
