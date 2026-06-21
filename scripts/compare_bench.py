#!/usr/bin/env python3
"""compare_bench.py — 跨平台基准测试对比

用法: python3 scripts/compare_bench.py baseline_master.tsv other_branch.tsv
输出: 数值差异 delta 表
"""

import sys

def parse_tsv(path):
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]
    data = {}
    for line in lines:
        parts = line.split("\t")
        if len(parts) < 7:
            continue
        name = parts[0]
        pitch, roll, yaw = float(parts[4]), float(parts[5]), float(parts[6])
        data[name] = (pitch, roll, yaw)
    return data

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 compare_bench.py baseline.tsv branch.tsv")
        sys.exit(1)
    base = parse_tsv(sys.argv[1])
    other = parse_tsv(sys.argv[2])
    print("filter\tpitch_delta\troll_delta\tyaw_delta")
    for name in sorted(base.keys()):
        if name not in other:
            print(f"{name}\tMISSING\t-\t-")
            continue
        bp, br, by = base[name]
        op, oro, oy = other[name]
        print(f"{name}\t{op-bp:.6f}\t{oro-br:.6f}\t{oy-by:.6f}")

if __name__ == "__main__":
    main()
