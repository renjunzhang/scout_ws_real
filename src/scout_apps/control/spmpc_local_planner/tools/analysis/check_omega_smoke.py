#!/usr/bin/env python3
"""新 omega-rate(6D/10D)模型 smoke 速查。

对 OUT_DIR 下每个 bag 报三件事，对应你的成功判据：
  不停滞 -> cmd_v@10s（≈0 即停滞）
  不抖   -> omega-rate p95 / max（新模型应 <= ~1.2，max 明显超 1.2 即异常）
  降晃   -> /spmpc/slosh_height 去前 2s 的 peak（与 §7 口径一致；仿真用模型 proxy，实物看 RGB）

用法:
  python3 check_omega_smoke.py <OUT_DIR>
"""
import sys
import os
import glob
import math
import collections
import numpy as np
import rosbag


def analyze(bag_path):
    cw = []; cv = []; ct = []
    osp = []; ot = []
    sh = []; sht = []
    status = collections.Counter()
    t0 = None
    with rosbag.Bag(bag_path) as bag:
        for tp, m, t in bag.read_messages(
                topics=["/cmd_vel", "/odom", "/spmpc/slosh_height", "/spmpc/status"]):
            ts = t.to_sec()
            t0 = ts if t0 is None else t0
            if tp == "/cmd_vel":
                cw.append(m.angular.z); cv.append(m.linear.x); ct.append(ts - t0)
            elif tp == "/odom":
                osp.append(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)); ot.append(ts - t0)
            elif tp == "/spmpc/slosh_height":   # 模型 proxy，单位 mm
                sh.append(m.data); sht.append(ts - t0)
            elif tp == "/spmpc/status":
                status[m.data] += 1
    r = {"status": "-", "cmd_v_10s": float('nan'),
         "wrate_p95": float('nan'), "wrate_max": float('nan'), "peak_ex2": float('nan')}
    if status:
        r["status"] = status.most_common(1)[0][0]
    if cv:
        cv = np.array(cv); ct = np.array(ct)
        r["cmd_v_10s"] = float(cv[ct > 10].mean()) if (ct > 10).any() else float(cv.mean())
    if len(cw) > 3:
        cw = np.array(cw); ctc = np.array(ct)
        dt = np.diff(ctc); good = (dt > 0.015) & (dt < 0.1)   # 丢时间戳毛刺
        wr = np.abs(np.diff(cw) / dt)[good]
        if len(wr):
            r["wrate_p95"] = float(np.percentile(wr, 95)); r["wrate_max"] = float(wr.max())
    tmove = 0.0
    if osp:
        osp = np.array(osp); ot = np.array(ot)
        mv = np.where(osp > 0.05)[0]
        if len(mv):
            tmove = ot[mv[0]]
    if sh:
        sh = np.array(sh); sht = np.array(sht)
        mask = sht > tmove + 2.0
        if mask.any():
            r["peak_ex2"] = float(sh[mask].max())
    return r


def main():
    if len(sys.argv) < 2:
        print("用法: python3 check_omega_smoke.py <OUT_DIR>"); sys.exit(1)
    root = sys.argv[1]
    bags = sorted(glob.glob(f"{root}/**/*.bag", recursive=True))
    if not bags:
        print(f"[warn] {root} 下没有 .bag"); sys.exit(0)
    print(f"{'bag':30} {'status':20} {'cmd_v@10s':>9} {'wrate_p95':>9} {'wrate_max':>9} {'slosh_pk_ex2(mm)':>16}  flags")
    for b in bags:
        r = analyze(b)
        name = os.path.basename(b).replace(".bag", "")[:30]
        flags = []
        if r["cmd_v_10s"] == r["cmd_v_10s"] and r["cmd_v_10s"] < 0.05:
            flags.append("停滞?")
        if r["wrate_max"] == r["wrate_max"] and r["wrate_max"] > 1.5:
            flags.append("超cap?")
        print(f"{name:30} {str(r['status'])[:20]:20} {r['cmd_v_10s']:9.3f} "
              f"{r['wrate_p95']:9.2f} {r['wrate_max']:9.2f} {r['peak_ex2']:16.1f}  {' '.join(flags)}")
    print("\n判读: 不停滞=cmd_v@10s 明显>0; 不抖=wrate_max 不超~1.2(新模型应如此); "
          "降晃=对比 B_slosh/B_ours 的 slosh_pk_ex2 是否 < B_smooth。")


if __name__ == "__main__":
    main()
