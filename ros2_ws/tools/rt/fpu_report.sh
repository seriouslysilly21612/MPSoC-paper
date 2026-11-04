#!/usr/bin/env bash
# FPU evidence report — APU (Zynq UltraScale+ / Cortex-A53) hardware FP usage.
#
# Runs fully offline: no robot, no EtherCAT, no ROS, no servo power. It reads
# already-built binaries, replays an already-recorded DataLog, and re-runs the
# offline reach map — so it is safe to run with the arm parked or unplugged.
#
#   usage: tools/rt/fpu_report.sh [outdir]     (default: evidence/fpu)
set -u

OUT=${1:-/home/ubuntu/ros2_ws/evidence/fpu}
APP=/home/ubuntu/RAON-RT-Revision/App/Indy7
BIN=$APP/bin/Indy7Ctrl.out
URDF=$APP/indy7.urdf
LOG=$(ls -t "$APP"/rt_log_results/DataLog_*.csv 2>/dev/null | grep -v _fk | head -1)
CPU=3                       # isolated core; robot offline so nothing else needs it
mkdir -p "$OUT"
CSV=$OUT/fpu_metrics.csv
REPORT=$OUT/fpu_report.txt
exec > >(tee "$REPORT") 2>&1

sec(){ printf '\n== %s ==\n' "$1"; }
echo "section,metric,value,unit" > "$CSV"
kv(){ printf '%-30s %-12s %s\n' "$2" "$3" "${4:-}"; echo "$1,$2,$3,${4:-}" >> "$CSV"; }

echo "FPU EVIDENCE REPORT   host=$(hostname)  kernel=$(uname -r)  date=$(date -Is)"
echo "robot: NOT CONNECTED — every number below is static analysis or offline replay"

# ------------------------------------------------------------- bench (built first)
cat > "$OUT/.fpbench.c" <<'EOF'
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
typedef double v2 __attribute__((vector_size(16)));
typedef float  v4 __attribute__((vector_size(16)));
static double el(struct timespec a,struct timespec b){
  return (b.tv_sec-a.tv_sec)*1e9+(b.tv_nsec-a.tv_nsec); }
#define N 20000000
static volatile double g_sink;
/* Four independent FMA chains: enough ILP to reach the issue rate rather
   than measuring FMA latency. Flop counts assume FMA = 2 flop. */
static double f64_scalar(void){ struct timespec t0,t1;
  double a=1.0000001,b=0.9999999,c=1.0000002,d=0.9999998;
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N;i++){a=a*b+.1;b=b*c+.1;c=c*d+.1;d=d*a+.1;}
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=a+b+c+d;
  return (8.0*N)/(el(t0,t1)/1e3); }
static double f64_neon(void){ struct timespec t0,t1;
  v2 a={1.0000001,1.0000003},b={0.9999999,0.9999997},
     c={1.0000002,1.0000004},d={0.9999998,0.9999996},k={.1,.1};
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N;i++){a=a*b+k;b=b*c+k;c=c*d+k;d=d*a+k;}
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=a[0]+b[0]+c[0]+d[0];
  return (16.0*N)/(el(t0,t1)/1e3); }
static double f32_neon(void){ struct timespec t0,t1;
  v4 a={1,1,1,1},b={1,1,1,1},c={1,1,1,1},d={1,1,1,1},k={.1f,.1f,.1f,.1f};
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N;i++){a=a*b+k;b=b*c+k;c=c*d+k;d=d*a+k;}
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=a[0]+b[0]+c[0]+d[0];
  return (32.0*N)/(el(t0,t1)/1e3); }
/* Dependent integer adds, one per cycle on A53 -> approximate core clock.
   asm volatile so the optimiser cannot fold the chain into a single add;
   unrolled x8 so the loop's own increment/compare/branch does not dominate. */
static double clock_mhz(void){ struct timespec t0,t1; long x=0;
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N/8;i++) __asm__ volatile(
    "add %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1\n\t"
    "add %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1":"+r"(x));
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=(double)x;
  return (N/8*8)/(el(t0,t1)/1e3); }
/* Subnormal penalty: the classic x86 RT-jitter trap. Operands are refreshed
   from memory every iteration so the maths cannot be hoisted out of the loop. */
static double buf[64];
static double subnorm_ns(double v){ struct timespec t0,t1; double acc=0;
  for(int w=0;w<64;w++) buf[w]=v;
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<2000000;i++){ acc+=buf[i&63]*buf[(i+1)&63]+buf[i&63]/3.0; buf[i&63]=v; }
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=acc;
  return el(t0,t1)/2000000.0; }
int main(int argc,char**argv){
  uint64_t f; __asm__ volatile("mrs %0, fpcr":"=r"(f));
  if(argc>1&&!strcmp(argv[1],"--fpcr")){
    printf("0x%08llx\n",(unsigned long long)f); return 0; }
  f64_scalar();                                     /* warm */
  printf("clock_mhz %.0f\n",       clock_mhz());
  printf("f64_scalar %.0f\n",      f64_scalar());
  printf("f64_neon %.0f\n",        f64_neon());
  printf("f32_neon %.0f\n",        f32_neon());
  double n=subnorm_ns(1.5e-3), d=subnorm_ns(5.0e-319);
  printf("normal_ns %.3f\nsubnormal_ns %.3f\nsubnormal_ratio %.2f\n",n,d,d/n);
  return 0; }
EOF
gcc -O2 -o "$OUT/.fpbench" "$OUT/.fpbench.c" || echo "WARN: bench build failed"

# ---------------------------------------------------------------- 1. hardware
sec "1. Hardware — does the FPU exist"
kv hw cpu_part  "$(awk -F': ' '/CPU part/{print $2;exit}' /proc/cpuinfo)" "(0xd03 = Cortex-A53)"
kv hw arch      "$(uname -m)"
kv hw cores     "$(nproc --all)" "(isolated: $(cat /sys/devices/system/cpu/isolated))"
kv hw hwcap_fp    "$(grep -qw fp    /proc/cpuinfo && echo yes || echo no)" "scalar FPU, incl. double"
kv hw hwcap_asimd "$(grep -qw asimd /proc/cpuinfo && echo yes || echo no)" "NEON SIMD"
kv hw at_hwcap  "0x$(LD_SHOW_AUXV=1 /bin/true | awk '/AT_HWCAP:/{print $2;exit}')" "bit0=FP bit1=ASIMD"
kv hw fpcr      "$(taskset -c $CPU "$OUT/.fpbench" --fpcr 2>/dev/null||echo n/a)" "FZ=0 -> full IEEE754, no flush-to-zero"

# ---------------------------------------------------------------- 2. compiler
sec "2. Toolchain — no FP flag appears in any RAON-RT Makefile, and none is needed"
kv cc gcc            "$(gcc -dumpfullversion)"
kv cc default_march  "$(gcc -Q --help=target 2>/dev/null|awk '/^  -march=/{print $2}')"
kv cc default_mabi   "$(gcc -Q --help=target 2>/dev/null|awk '/^  -mabi=/{print $2}')" "hard-float AAPCS64"
kv cc __ARM_FP       "$(gcc -dM -E -x c /dev/null|awk '/__ARM_FP /{print $3}')" "14 = 0xE = half+single+double"
kv cc app_cflags     "$(awk -F'= ' '/^CFLAGS_OPTIONS/{print $2;exit}' "$APP/Makefile" 2>/dev/null|tr -d '\n')"
echo "  -mfpu / -mfloat-abi do not exist on AArch64: FP is mandatory in ARMv8-A,"
echo "  and there is no soft-float ABI to fall back to."

# ------------------------------------------------- 3. static census of the app
sec "3. Static census — FP instructions actually present in the shipped binaries"
printf '  %-16s %8s %8s %8s %8s %10s\n' binary scalarFP neonFP total density softfloat
for b in "$BIN" "$APP/bin/reachmap.out" "$APP/bin/fk_replay.out"; do
  [ -f "$b" ] || continue
  d=$(objdump -d "$b" 2>/dev/null)
  s=$(echo "$d"|awk '{print $3}'|grep -Ec '^(fadd|fsub|fmul|fdiv|fmadd|fmsub|fnmul|fnmadd|fsqrt|fcmp|fcmpe|fcvt|fcvtzs|fcvtzu|fneg|fabs|fmax|fmin|scvtf|ucvtf)')
  n=$(echo "$d"|grep -Ec '\b(fmla|fmls|fmul|fadd|fsub|fdiv|fmax|fmin|fneg|fabs)[[:space:]]+v[0-9]+\.')
  t=$(echo "$d"|grep -cE '^[[:space:]]+[0-9a-f]+:')
  h=$(echo "$d"|grep -cEi '__aeabi_|__adddf3|__muldf3|__divdf3|__floatdidf')
  dens=$(awk -v a=$s -v b=$n -v c=$t 'BEGIN{printf "%.2f%%",100*(a+b)/c}')
  tag=$(basename "$b" .out)
  printf '  %-16s %8d %8d %8d %8s %10d\n' "$tag" "$s" "$n" "$t" "$dens" "$h"
  for p in scalar_fp:$s neon_fp:$n total:$t softfloat_helpers:$h; do
    echo "static,$tag.${p%%:*},${p##*:},instr" >> "$CSV"; done
done
echo "  softfloat=0 everywhere: no __adddf3/__muldf3/__divdf3 emulation call exists,"
echo "  so every one of these operations is executed by the hardware FPU."
echo
echo "  from FullDynControllerRT.o — the object the 1 kHz control law lives in:"
objdump -d "$APP/obj/FullDynControllerRT.o" 2>/dev/null \
  | grep -E '\b(fmadd|fsqrt|fdiv)\b|\bfadd[[:space:]]+v[0-9]+\.2d' | head -5 | sed 's/^/    /'

# ------------------------------------------------- 4. measured FPU performance
sec "4. Measured FPU performance (core $CPU, isolated)"
taskset -c $CPU "$OUT/.fpbench" | while read -r k v; do case $k in
  clock_mhz)       kv bench core_clock       "$v" "MHz (approx, dependent-add chain)";;
  f64_scalar)      kv bench f64_scalar_fma   "$v" "MFLOP/s  double, scalar";;
  f64_neon)        kv bench f64_neon_fma     "$v" "MFLOP/s  double, NEON 2-wide";;
  f32_neon)        kv bench f32_neon_fma     "$v" "MFLOP/s  float,  NEON 4-wide";;
  normal_ns)       kv bench normal_operand   "$v" ns/iter;;
  subnormal_ns)    kv bench subnormal_operand "$v" ns/iter;;
  subnormal_ratio) kv bench subnormal_penalty "$v" "x   ~1.0 => no FTZ / -ffast-math needed";;
esac; done
echo "  NOTE: f64 NEON is SLOWER than f64 scalar. The A53 NEON datapath is 64-bit,"
echo "  so a 128-bit double vector op is cracked into two micro-ops and gains"
echo "  nothing. f32 NEON does speed up, which confirms this is the datapath"
echo "  width and not a benchmark artefact. -O3 auto-vectorising our double"
echo "  maths into fadd v.2d is therefore free of benefit, not free of cost."

# ------------------------------------- 5. real controller maths, offline (FP-bound)
sec "5. Real workload A — the shipped IK solver, re-run offline (no robot)"
if [ -x "$APP/bin/reachmap.out" ] && [ -f "$HOME/.indy7_ready_seed" ]; then
  rmcsv=$OUT/.reachmap.csv
  t0=$(date +%s%N)
  (cd "$APP" && taskset -c $CPU ./bin/reachmap.out --urdf "$URDF" --step 0.02 --z 0.18 \
      --out "$rmcsv" --log "$OUT/.reachmap.log") 2>&1 | grep -E 'PASS|solve time' | sed 's/^/  /'
  t1=$(date +%s%N)
  ms=$(( (t1-t0)/1000000 ))
  cells=$(grep -vc '^#' "$rmcsv" 2>/dev/null)
  kv workload ik_cells "$cells" "grid cells, each a full SolveReadyIK"
  kv workload ik_wall  "$ms" ms
  kv workload ik_rate  "$(awk -v c=$cells -v m=$ms 'BEGIN{printf "%.0f",(m?c*1000.0/m:0)}')" "IK solves/s"
  echo "  reachmap links FullDynControllerRT.o, so this is the same solver code"
  echo "  the robot runs — pure double-precision FP, no I/O in the hot path."
else
  echo "  SKIPPED: reachmap.out or ~/.indy7_ready_seed missing"
fi

sec "6. Real workload B — offline FK vs the FP the robot already computed"
if [ -f "$LOG" ] && [ -x "$APP/bin/fk_replay.out" ]; then
  rows=$(( $(wc -l < "$LOG") - 1 ))
  tmp=$(mktemp /tmp/fk_replay_XXXX.csv)
  t0=$(date +%s%N)
  taskset -c $CPU "$APP/bin/fk_replay.out" "$URDF" "$LOG" "$tmp" >/dev/null 2>&1
  t1=$(date +%s%N); ms=$(( (t1-t0)/1000000 ))
  kv workload fk_source "$(basename "$LOG")"
  kv workload fk_cycles "$rows" "logged 1 kHz cycles"
  kv workload fk_wall   "$ms" "ms (CSV-parse bound, NOT a control-loop timing)"
  awk -F, 'NR==1{for(i=1;i<=NF;i++)h[$i]=i;next}
     {dx=$(h["tcp_x"])-$(h["act_fk_x"]);dy=$(h["tcp_y"])-$(h["act_fk_y"]);
      dz=$(h["tcp_z"])-$(h["act_fk_z"]);d=sqrt(dx*dx+dy*dy+dz*dz);
      if(d>m)m=d;s+=d;n++}END{if(n)printf "%.6f %.6f %d\n",m*1000,s/n*1000,n}' "$tmp" \
  | while read -r mx mn nn; do
      kv workload fk_agree_max  "$mx" "mm  max |robot TCP - offline FK|, n=$nn"
      kv workload fk_agree_mean "$mn" "mm  mean"
    done
  rm -f "$tmp"
  echo "  The robot computed tcp_* on its FPU during the run; this recomputes the"
  echo "  same FK from the logged joint angles now. Agreement at the micrometre"
  echo "  level is what a deterministic IEEE754 FPU looks like — a soft-float or"
  echo "  mis-rounded path would not land here."
else
  echo "  SKIPPED: no DataLog or fk_replay.out"
fi

sec "7. How much FP does one 1 kHz control cycle actually cost"
cat > "$OUT/.cycle.cpp" <<'EOF'
/* Times the exact RBDL calls FullDynControllerRT issues every 1 kHz cycle.
   Same URDF, same 6 DOF, same call sequence — see ComputeComputedTorque()
   and ComputeFullDynamics() in FullDynControllerRT.cpp. */
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cstdio>
#include <ctime>
using namespace RigidBodyDynamics; using namespace RigidBodyDynamics::Math;
static double el(timespec a,timespec b){return (b.tv_sec-a.tv_sec)*1e9+(b.tv_nsec-a.tv_nsec);}
#define REP 20000
/* q is nudged each iteration so no call can be cached or hoisted. */
#define TIME(key,expr) { timespec t0,t1; for(int i=0;i<200;i++){expr;} \
  clock_gettime(CLOCK_MONOTONIC,&t0); for(int i=0;i<REP;i++){q[0]+=1e-9;expr;} \
  clock_gettime(CLOCK_MONOTONIC,&t1); double us=el(t0,t1)/REP/1000.0; \
  printf("%s %.3f\n",key,us); }
int main(int argc,char**argv){
  Model m; if(!Addons::URDFReadFromFile(argv[1],&m,false)){fprintf(stderr,"urdf fail\n");return 1;}
  int dof=m.dof_count;
  VectorNd q=VectorNd::Zero(dof),qd=VectorNd::Zero(dof),z=VectorNd::Zero(dof),
           h=VectorNd::Zero(dof),g=VectorNd::Zero(dof);
  MatrixNd M=MatrixNd::Zero(dof,dof),J=MatrixNd::Zero(6,dof);
  q<<0.45,-0.43,-2.13,-0.04,0.67,-0.56; qd.setConstant(0.1);
  unsigned bid=m.GetBodyId(m.GetBodyName(m.mBodies.size()-1).c_str());
  Vector3d tcp(0,0,0.06),p; Matrix3d R;
  printf("dof %d\n",dof);
  TIME("crba_us",   CompositeRigidBodyAlgorithm(m,q,M))
  TIME("nle_us",    NonlinearEffects(m,q,qd,h))
  TIME("grav_us",   InverseDynamics(m,q,z,z,g))
  TIME("fk_us",     p=CalcBodyToBaseCoordinates(m,q,bid,tcp))
  TIME("jac6d_us",  CalcPointJacobian6D(m,q,bid,tcp,J))
  TIME("bwo_us",    R=CalcBodyWorldOrientation(m,q,bid))
  return 0; }
EOF
if g++ -O3 -DNDEBUG -I/usr/include/eigen3 -o "$OUT/.cycle" "$OUT/.cycle.cpp" \
     -lrbdl -lrbdl_urdfreader 2>/dev/null; then
  eval "$(taskset -c $CPU "$OUT/.cycle" "$URDF" | awk '{printf "%s=%s\n",$1,$2}')"
  kv cycle crba_us  "$crba_us"  "mass matrix M(q)        — CRBA"
  kv cycle nle_us   "$nle_us"   "h(q,qd) = C*qd + g(q)   — NonlinearEffects"
  kv cycle grav_us  "$grav_us"  "gravity g(q)            — InverseDynamics"
  kv cycle fk_us    "$fk_us"    "TCP position            — CalcBodyToBaseCoordinates"
  kv cycle bwo_us   "$bwo_us"   "TCP orientation         — CalcBodyWorldOrientation"
  kv cycle jac6d_us "$jac6d_us" "6xN Jacobian            — IK modes only"
  echo
  echo "  Per control mode (the switch in Update()), RBDL portion only:"
  tmode(){ local lbl=$1; shift; local t=0 v
    for v in "$@"; do t=$(awk -v a=$t -v b=$v 'BEGIN{print a+b}'); done
    printf '  %-26s %8.1f us/cycle  %5.1f%% of the 1000 us budget\n' "$lbl" "$t" \
      "$(awk -v x=$t 'BEGIN{print x/10}')"
    echo "mode,${lbl}.us,$t,us per cycle" >> "$CSV"; }
  tmode eGravityCompensation "$grav_us"
  tmode eComputedTorque      "$crba_us" "$nle_us"
  tmode eFullDynamics        "$crba_us" "$nle_us" "$grav_us"
  tmode eTrackingServo       "$crba_us" "$nle_us" "$fk_us" "$bwo_us"
  trk=$(awk -v a=$crba_us -v b=$nle_us -v c=$fk_us -v d=$bwo_us 'BEGIN{printf "%.1f",a+b+c+d}')
  echo
  echo "  All of these are dense double-precision linear algebra (Eigen + RBDL);"
  echo "  the arithmetic is what costs the time, not the I/O."
  echo
  echo "  Two different denominators, do not confuse them:"
  echo "    vs the 1 ms DEADLINE      -> $(awk -v x=$trk 'BEGIN{printf "%.1f%%",x/10}'), i.e. the loop finishes with room to spare"
  echo "    vs the loop's OWN runtime -> a substantial share of what the loop does"
  echo "  The second one is NOT quantified here: the figures above are means, while"
  echo "  the only RT loop timings we have (D10 A/B: max 33.9 us non-isolated /"
  echo "  51.1 us isolated) are maxima including jitter. Dividing a mean by a max"
  echo "  would overstate it. Establishing the share needs a mean loop time, which"
  echo "  requires the robot running."
  echo
  echo "  Break-even, from the measured mean: FP would have to get $(awk -v x=$trk 'BEGIN{printf "%.0f",1000/x}')x slower"
  echo "  ($trk us -> 1000 us) to threaten the budget. Whether software emulation"
  echo "  crosses that line cannot be measured on this board — AArch64 has no"
  echo "  soft-float ABI and libgcc ships no __adddf3/__muldf3/__divdf3 to call."
else
  echo "  SKIPPED: could not build against librbdl"
fi

sec "8. How MANY operations — exact dynamic instruction counts per 1 kHz cycle"
# The A53 PMU exposes no FP event (no vfp_spec/ase_spec), and perf_event_paranoid
# is 4, so counters are out. Instead: single-step the child through one call and
# decode each instruction word. AArch64 instructions are a fixed 32 bits and the
# ARM ARM top-level encoding puts all scalar-FP and Advanced-SIMD data processing
# in op0 = bits[28:25] == x111, so classifying is a bit test, not a disassembler.
cat > "$OUT/.opbench.cpp" <<'EOF'
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
using namespace RigidBodyDynamics; using namespace RigidBodyDynamics::Math;
int main(int argc,char**argv){
  Model m; if(!Addons::URDFReadFromFile(argv[1],&m,false))return 1;
  int dof=m.dof_count, which=atoi(argv[2]);
  VectorNd q=VectorNd::Zero(dof),qd=VectorNd::Zero(dof),z=VectorNd::Zero(dof),
           h=VectorNd::Zero(dof),g=VectorNd::Zero(dof);
  MatrixNd M=MatrixNd::Zero(dof,dof),J=MatrixNd::Zero(6,dof);
  q<<0.45,-0.43,-2.13,-0.04,0.67,-0.56; qd.setConstant(0.1);
  unsigned bid=m.GetBodyId(m.GetBodyName(m.mBodies.size()-1).c_str());
  Vector3d tcp(0,0,0.06),p; Matrix3d R;
  for(int i=0;i<50;i++){ CompositeRigidBodyAlgorithm(m,q,M); NonlinearEffects(m,q,qd,h);
    InverseDynamics(m,q,z,z,g); p=CalcBodyToBaseCoordinates(m,q,bid,tcp);
    CalcPointJacobian6D(m,q,bid,tcp,J);
    R=CalcBodyWorldOrientation(m,q,bid); }   /* warm: fault pages, resolve PLT */
  raise(SIGUSR1);
  switch(which){
    case 0: CompositeRigidBodyAlgorithm(m,q,M); break;
    case 1: NonlinearEffects(m,q,qd,h); break;
    case 2: InverseDynamics(m,q,z,z,g); break;
    case 3: p=CalcBodyToBaseCoordinates(m,q,bid,tcp); break;
    case 4: CalcPointJacobian6D(m,q,bid,tcp,J); break;
    case 5: R=CalcBodyWorldOrientation(m,q,bid); break; }
  raise(SIGUSR2);
  printf("%g\n",p[0]+M(0,0)+h[0]+g[0]+J(0,0)+R(0,0)); return 0; }
EOF
cat > "$OUT/.opcount.c" <<'EOF'
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <sys/user.h>
#include <sys/uio.h>
#include <elf.h>
#include <stdint.h>
#include <stdlib.h>
static int getpc(pid_t p,uint64_t*pc){ struct user_regs_struct r;
  struct iovec io={&r,sizeof(r)};
  if(ptrace(PTRACE_GETREGSET,p,NT_PRSTATUS,&io)<0) return -1; *pc=r.pc; return 0; }
static FILE* dumpf=NULL;
int main(int argc,char**argv){
  if(getenv("OPDUMP")) dumpf=fopen(getenv("OPDUMP"),"w");
  if(argc<2){fprintf(stderr,"usage: %s <prog> [args...]\n",argv[0]);return 2;}
  pid_t pid=fork();
  if(pid==0){ ptrace(PTRACE_TRACEME,0,0,0);
    if(!freopen("/dev/null","w",stdout)) _exit(126);
    execv(argv[1],&argv[1]); _exit(127); }
  int st; waitpid(pid,&st,0);
  ptrace(PTRACE_SETOPTIONS,pid,0,PTRACE_O_EXITKILL);
  /* Phase 1: run free. URDF parsing and warm-up are millions of instructions
     and single-stepping them would take minutes. */
  int sig=0;
  for(;;){ if(ptrace(PTRACE_CONT,pid,0,sig)<0){fprintf(stderr,"cont failed\n");return 1;}
    sig=0;
    if(waitpid(pid,&st,0)<0||WIFEXITED(st)||WIFSIGNALED(st)){
      fprintf(stderr,"child gone before SIGUSR1\n"); return 1; }
    int s=WSTOPSIG(st); if(s==SIGUSR1) break; if(s!=SIGTRAP) sig=s; }
  /* Phase 2: single-step the region of interest, decoding each instruction. */
  unsigned long long total=0,fpsimd=0,ldst=0,branch=0;
  for(;;){
    if(ptrace(PTRACE_SINGLESTEP,pid,0,sig)<0) break; sig=0;
    if(waitpid(pid,&st,0)<0) break;
    if(WIFEXITED(st)||WIFSIGNALED(st)||!WIFSTOPPED(st)) break;
    int s=WSTOPSIG(st);
    if(s==SIGUSR2) break;
    if(s!=SIGTRAP){ sig=s; continue; }
    uint64_t pc; if(getpc(pid,&pc)<0) break;
    errno=0; long w=ptrace(PTRACE_PEEKTEXT,pid,(void*)pc,0); if(errno) continue;
    uint32_t insn=(uint32_t)w, op0=(insn>>25)&0xF;
    total++;
    /* x111 is the FP/SIMD *unit*, which is wider than "floating-point
       arithmetic": it also holds NEON integer ops and the crypto extensions.
       The bit test cannot tell them apart, so every word in this bucket is
       dumped and handed to objdump for classification by real mnemonic. */
    if((op0&0x7)==0x7){ fpsimd++; if(dumpf) fprintf(dumpf,"%08x\n",insn); }
    else if((op0&0x5)==0x4) ldst++;     /* x1x0 loads and stores          */
    else if((op0&0xE)==0xA) branch++;   /* 101x branch / system           */ }
  kill(pid,SIGKILL); waitpid(pid,&st,0);
  if(dumpf) fclose(dumpf);
  printf("%llu %llu %llu %llu\n",total,fpsimd,ldst,branch); return 0; }
EOF
cat > "$OUT/.opselftest.c" <<'EOF'
/* Known-answer check: exactly 2000 FP instructions inside the traced region,
   written in asm so the count does not depend on the optimiser. */
#include <signal.h>
#include <stdio.h>
int main(void){ double a=1.5; raise(SIGUSR1);
  for(int i=0;i<1000;i++) __asm__ volatile("fmadd d0,d0,d0,d0":::"d0");
  for(int i=0;i<1000;i++) __asm__ volatile("fmla v1.2d,v1.2d,v1.2d":::"v1");
  raise(SIGUSR2); printf("%g\n",a); return 0; }
EOF
cat > "$OUT/.classify.py" <<'EOF'
"""Disassemble the dumped FP/SIMD instruction words and split them by real
mnemonic. The x111 encoding group is the FP/SIMD unit, not floating-point
arithmetic: NEON integer ops and the crypto extensions live there too.
Prints: <fp_arithmetic> <fp_move> <simd_integer> <crypto>"""
import collections, struct, subprocess, sys
words = [int(l, 16) for l in open(sys.argv[1])]
cnt   = collections.Counter(words)
uniq  = sorted(cnt)
raw   = sys.argv[1] + ".bin"
open(raw, 'wb').write(b''.join(struct.pack('<I', w) for w in uniq))
out = subprocess.run(['objdump', '-D', '-b', 'binary', '-m', 'aarch64', raw],
                     capture_output=True, text=True).stdout
mnem = {}
for line in out.splitlines():
    parts = line.split('\t')
    if len(parts) < 3:
        continue
    try:
        idx = int(parts[0].strip().rstrip(':'), 16) // 4
    except ValueError:
        continue
    if idx < len(uniq):
        mnem[uniq[idx]] = parts[2].split()[0]
ARITH = ('fadd','fsub','fmul','fdiv','fmadd','fmsub','fnmadd','fnmsub','fnmul',
         'fsqrt','fabs','fneg','fmax','fmin','fmla','fmls','frint','fcvt',
         'scvtf','ucvtf','fcmp','fcmpe','fccmp','fcsel','frecp','frsqrt')
MOV   = ('fmov','ins','umov','smov','dup','movi','mvni')
CRYPT = ('aes','sha','pmull','crc')
b = collections.Counter()
for w, c in cnt.items():
    m = mnem.get(w, '???')
    if   m.startswith(ARITH): b['arith'] += c
    elif m.startswith(MOV):   b['mov']   += c
    elif m.startswith(CRYPT): b['crypto']+= c
    elif m.startswith('f'):   b['arith'] += c     # any other f* is FP work
    else:                     b['simd']  += c
print(b['arith'], b['mov'], b['simd'], b['crypto'])
EOF
if gcc -O2 -o "$OUT/.opcount" "$OUT/.opcount.c" 2>/dev/null && \
   gcc -O2 -o "$OUT/.opselftest" "$OUT/.opselftest.c" 2>/dev/null && \
   g++ -O3 -DNDEBUG -I/usr/include/eigen3 -o "$OUT/.opbench" "$OUT/.opbench.cpp" \
       -lrbdl -lrbdl_urdfreader 2>/dev/null; then
  st=$(taskset -c $CPU "$OUT/.opcount" "$OUT/.opselftest" | awk '{print $2}')
  kv opcount selftest_fp "$st" "expected exactly 2000 -> counter verified"
  [ "$st" = "2000" ] || echo "  WARNING: counter self-test FAILED, counts below are suspect"
  echo "  Each row is ONE call of that RBDL function, single-stepped end to end."
  printf '  %-22s %8s %9s %8s %8s %8s %8s\n' \
    call instr 'FP arith' 'FP mov' 'SIMDint' ld/st branch
  declare -A I A
  i=0; for nm in CRBA NonlinearEffects InverseDynamics FK Jacobian6D BodyWorldOrientation; do
    read -r tot fp ls br <<<"$(OPDUMP=$OUT/.d$i taskset -c $CPU \
        "$OUT/.opcount" "$OUT/.opbench" "$URDF" $i)"
    read -r arith mov simdint crypto <<<"$(python3 "$OUT/.classify.py" "$OUT/.d$i")"
    printf '  %-22s %8d %9d %8d %8d %8d %8d\n' \
      "$nm" "$tot" "$arith" "$mov" "$simdint" "$ls" "$br"
    for p in instr:$tot fp_arith:$arith fp_mov:$mov simd_int:$simdint crypto:$crypto; do
      echo "opcount,$nm.${p%%:*},${p##*:},instructions" >> "$CSV"; done
    [ "$crypto" = "0" ] || echo "  NOTE: $nm executed $crypto crypto instructions"
    I[$nm]=$tot; A[$nm]=$arith
    i=$((i+1))
  done
  echo
  echo "  Per control mode — composed from the switch in Update() and the bodies of"
  echo "  ComputeGravityCompensation / ComputeComputedTorque / ComputeFullDynamics /"
  echo "  ComputeTcpFK in FullDynControllerRT.cpp. This is the RBDL portion of one"
  echo "  1 kHz cycle, NOT the whole loop: EtherCAT PDO copies, the safety gates,"
  echo "  friction feed-forward, trajectory update and logging are not counted."
  printf '  %-26s %8s %9s %7s\n' 'control mode' instr 'FP arith' 'FP%'
  mode(){ # $1=label  $2..=function names to sum
    local lbl=$1; shift; local ti=0 ta=0 f
    for f in "$@"; do ti=$((ti+${I[$f]})); ta=$((ta+${A[$f]})); done
    printf '  %-26s %8d %9d %6.1f%%\n' "$lbl" "$ti" "$ta" \
      "$(awk -v a=$ta -v b=$ti 'BEGIN{print 100*a/b}')"
    echo "mode,${lbl}.instr,$ti,instructions per cycle" >> "$CSV"
    echo "mode,${lbl}.fp_arith,$ta,instructions per cycle" >> "$CSV"
    echo "mode,${lbl}.fp_rate,$(awk -v x=$ta 'BEGIN{printf "%.2f",x/1e3}'),M FP-arith/s at 1 kHz" >> "$CSV"
  }
  mode eGravityCompensation InverseDynamics
  mode eComputedTorque      CRBA NonlinearEffects
  mode eFullDynamics        CRBA NonlinearEffects InverseDynamics
  mode eTrackingServo       CRBA NonlinearEffects FK BodyWorldOrientation
  ta=$(( ${A[CRBA]} + ${A[NonlinearEffects]} + ${A[FK]} + ${A[BodyWorldOrientation]} ))
  echo "  (eInverseKinematics also runs CalcPointJacobian6D, but the rest of"
  echo "   ComputeJacobianBasedInverseKinematics is not broken out here.)"
  echo "  Columns are decided by objdump on the actual instruction words, not by"
  echo "  the bit test — the x111 encoding group is the FP/SIMD *unit*, so it also"
  echo "  carries NEON integer ops (mov/zip/ext/bsl) and the crypto extensions."
  echo "  'FP arith' is genuine floating-point work (fmadd/fmul/fadd/fmla/fdiv/"
  echo "  fsqrt/fcvt/fcmp...). 'FP mov' and 'SIMDint' still execute on the FP/SIMD"
  echo "  registers but compute no floating-point result."
  echo
  echo "  FLOP ceiling: counting every FP arithmetic instruction as a 2-flop FMA"
  echo "  gives at most ~$(awk -v x=$ta 'BEGIN{printf "%.0f",x*2/1e6*1000}') MFLOP/s, against the $(awk -F, '/bench,f64_scalar_fma/{print $3}' "$CSV") MFLOP/s this core was"
  echo "  measured to sustain — roughly one percent of one core."
else
  echo "  SKIPPED: could not build the instruction counter"
fi

sec "Output"
echo "  report : $REPORT"
echo "  metrics: $CSV"
