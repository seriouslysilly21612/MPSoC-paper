#!/usr/bin/env python3
"""count_periods.py -- the instructions that the R5 executes in each period of test/shadow_replay.cpp.

  qemu-arm -cpu cortex-r5f -one-insn-per-tb -d exec,nochain shadow_replay.elf FILE 1e-9 N 2>&1 >/dev/null |
      python3 -I test/count_periods.py <address of mark, hex>          (test/shadow_replay.sh with COUNT=N does it)

The trace has one line per instruction executed.  mark() is passed before and after every period, so the lines
between an odd and the next even pass are one period.  An instruction count is not a time: it has no cache miss
and no pipeline in it.
"""
import sys


def count(lines, mark):
    """the instruction counts of the periods: the lines between the passes 1-2, 3-4, ... through `mark`"""
    periods, n, begin = [], 0, None
    for line in lines:
        if not line.startswith(b"Trace"):
            continue
        n += 1
        i = line.find(b"/")
        j = line.find(b"/", i + 1)
        if int(line[i + 1:j], 16) != mark:
            continue
        if begin is None:
            begin = n
        else:
            periods.append(n - begin - 1)       # without the line of the second pass itself
            begin = None
    return periods


def main():
    mark = int(sys.argv[1], 16) & ~1
    p = count(sys.stdin.buffer, mark)
    if not p:
        sys.exit("count_periods: no period in the trace")
    s = sorted(p)
    top = sorted(range(len(p)), key=lambda k: -p[k])[:5]
    print("%d periods: instructions mean %.0f, median %d, 99 %% %d, max %d" %
          (len(p), sum(p) / len(p), s[len(s) // 2], s[min(len(s) - 1, len(s) * 99 // 100)], s[-1]))
    print("the five largest: " + ", ".join("period %d: %d" % (k, p[k]) for k in top))


if __name__ == "__main__":
    main()
