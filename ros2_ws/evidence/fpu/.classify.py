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
