#!/usr/bin/env python3
"""读 pos_watch.csv（stdin）压成对照表。★ 别再猜列号 —— 先读表头。

表头（19 列）：
  t_wall,t_ros,vio_x,vio_y,vio_z,|vio|,|v_vio|,prop_x,prop_y,prop_z,|prop|,
  pn_x,pn_y,pn_z,ad_x,ad_y,ad_z,acc_norm,gyro_norm

两个都叫"纯 IMU"的量，别混：
  · |prop| (第 11 列)     —— relay 自己用 IMU 推的
  · pn_y   (第 13 列)     —— VINS 的 /diag/pos_norm 里的纯 IMU 路径
  ★ 与 A 轮记录的 796.2 / 809.2 可比的是 **pn_x / pn_y**，不是 |prop|。
  A 轮就是拿这两个数的比值（1.6%）说明"视觉几乎没贡献"。

att_delta（第 15/16 列）★ 单位已经是度，不要再 math.degrees()。
"""
import sys
import datetime

HDR = ("t_wall,t_ros,vio_x,vio_y,vio_z,|vio|,|v_vio|,prop_x,prop_y,prop_z,|prop|,"
       "pn_x,pn_y,pn_z,ad_x,ad_y,ad_z,acc_norm,gyro_norm").split(",")


def main():
    rows = []
    for line in sys.stdin:
        line = line.strip()
        if not line or line.startswith("t_wall"):
            continue
        p = line.split(",")
        if len(p) < 19:
            continue
        try:
            rows.append([float(x) for x in p])
        except ValueError:
            continue
    if not rows:
        print("（CSV 里还没有数据）")
        return

    # ★ 判据是"视觉拉回比例"，不是原始比值：
    #     拉回 = (|纯IMU| − |VIO|) / |纯IMU|
    #   0%   = VIO 完全跟着纯 IMU 走 ⇒ **视觉什么都没做**（A 轮 = 1.6%）
    #   100% = VIO 被摁在 0 附近，纯 IMU 自己跑 ⇒ 视觉在干活
    def pull(vio, prop):
        return ((prop - vio) / prop * 100) if prop > 1e-9 else float("nan")

    print("%-9s %12s %12s %9s %10s %10s %9s %8s" %
          ("时间", "|VIO|(pn_x)", "|纯IMU|(pn_y)", "视觉拉回", "att_roll", "att_pitch", "|v_VIO|", "acc"))
    for r in rows[:: max(1, len(rows) // 24)]:          # 最多 24 行
        t = datetime.datetime.fromtimestamp(r[0], datetime.timezone.utc).strftime("%H:%M:%S")
        vio, prop = r[11], r[12]                        # pn_x, pn_y
        print("%-9s %12.4f %12.4f %8.1f%% %10.3f %10.3f %9.4f %8.2f" %
              (t, vio, prop, pull(vio, prop), r[14], r[15], r[6], r[17]))
    last = rows[-1]
    t = datetime.datetime.fromtimestamp(last[0], datetime.timezone.utc).strftime("%H:%M:%S")
    print("\n最后一行 %s：共 %d 行" % (t, len(rows)))
    print("  |VIO| = %.4f m   |纯IMU| = %.4f m   视觉拉回 = %.2f%%"
          % (last[11], last[12], pull(last[11], last[12])))
    print("  ★ 对照：A 轮视觉拉回 = 1.6%%（≈什么都没做）；本轮若明显更大，说明视觉真在起作用")


if __name__ == "__main__":
    main()
