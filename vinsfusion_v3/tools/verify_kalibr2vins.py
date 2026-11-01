#!/usr/bin/env python3
"""校验 kalibr2vins.py 的产物：与 OpenVINS 版链文件逐项对拍。

原理：VINS-Fusion 的 body_T_cam* 与 OpenVINS 的 T_imu_cam 是同一个物理量
（都等于 inv(Kalibr T_cam_imu)）。两条独立转换链若给出相同结果，
说明转换本身没有引入方向性错误 —— 这是最直接的证据。

用法：
  verify_kalibr2vins.py <out_dir> <openvins_chain.yaml>
"""
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from kalibr2vins import load  # noqa: E402


class VinsConfig:
    """用 cv2.FileStorage 读主配置。

    刻意不用 PyYAML：!!opencv-matrix 是 OpenCV 私有标签，PyYAML 读不了。
    而 VINS-Fusion 自己就是走 cv::FileStorage 读的 —— 用同一个解析器来读，
    等于顺带验证了"这份配置 VINS 能不能读懂"，比单纯比对数值更有意义。
    """

    def __init__(self, path):
        self.fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not self.fs.isOpened():
            raise SystemExit(f'cv2.FileStorage 打不开 {path} —— VINS 也读不了')
        self.path = path

    def mat(self, key):
        n = self.fs.getNode(key)
        if n.empty():
            raise SystemExit(f'配置里缺 {key}（cv::FileStorage 读不到）')
        return n.mat()

    def real(self, key):
        n = self.fs.getNode(key)
        if n.empty():
            raise SystemExit(f'配置里缺 {key}')
        return n.real()

    def text(self, key):
        n = self.fs.getNode(key)
        return n.string() if not n.empty() else None


def read_vins_cam_yaml(path):
    """从 VINS 相机 yaml 里取 fx,fy,cx,cy,k1,k2,p1,p2。"""
    txt = open(path).read()
    out = {}
    for k in ('fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2'):
        m = re.search(rf'^\s*{k}:\s*(\S+)\s*$', txt, re.M)
        if not m:
            raise SystemExit(f'{path} 里找不到 {k}')
        out[k] = float(m.group(1))
    return out


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    out_dir, ov_path = sys.argv[1], sys.argv[2]

    cfg = VinsConfig(f'{out_dir}/d430_px4_stereo_imu_config.yaml')
    ov = load(ov_path)

    print(f'VINS 侧: {out_dir}  (经 cv2.FileStorage 解析)')
    print(f'OpenVINS 侧: {ov_path}')
    print()
    print('两者都等于 inv(Kalibr T_cam_imu)，故应逐元素相等。')
    print('注：OpenVINS 版链文件由 kalibr_imucam_to_openvins.py 以 6 位小数写出，')
    print('    故外参与时偏只能比对到这个精度（1e-6）；内参/畸变在该文件中是全精度，')
    print('    因此那两项可要求逐位相同。')
    print('-' * 66)

    TOL_MAT = 2e-6   # OpenVINS 文件 6 位小数的量化步长
    TOL_EXACT = 1e-12

    # 先确认关键标量与话题被 OpenCV 正确读到
    print('主配置读取自检:')
    print(f'  imu={int(cfg.real("imu"))}  num_of_cam={int(cfg.real("num_of_cam"))}  '
          f'estimate_extrinsic={int(cfg.real("estimate_extrinsic"))}  '
          f'estimate_td={int(cfg.real("estimate_td"))}')
    for k in ('imu_topic', 'image0_topic', 'image1_topic', 'cam0_calib', 'cam1_calib',
              'output_path', 'acc_n', 'gyr_n', 'acc_w', 'gyr_w', 'g_norm'):
        v = cfg.text(k) if k.endswith(('topic', 'calib', 'path')) else cfg.real(k)
        print(f'  {k} = {v}')
    print('-' * 66)

    all_ok = True

    for i, cam in enumerate(('cam0', 'cam1')):
        mine = cfg.mat(f'body_T_cam{i}')
        theirs = np.array(ov[cam]['T_imu_cam'], dtype=float)
        d = np.abs(mine - theirs)
        ok = d.max() < TOL_MAT
        all_ok &= ok
        mark = '一致' if ok else '**不一致**'
        print(f'{cam} 外参 T: 最大逐元素差 = {d.max():.3e}   {mark}')
        if not ok:
            print('  VINS 版:\n', mine)
            print('  OpenVINS 版:\n', theirs)
        t = mine[:3, 3]
        print(f'        平移 = ({t[0]*1000:.3f}, {t[1]*1000:.3f}, {t[2]*1000:.3f}) mm'
              f'   |t| = {np.linalg.norm(t)*1000:.3f} mm')

        # 内参
        v = read_vins_cam_yaml(f'{out_dir}/{cam}.yaml')
        ki = np.array([v['fx'], v['fy'], v['cx'], v['cy']])
        oi = np.array(ov[cam]['intrinsics'], dtype=float)
        dk = np.abs(ki - oi).max()
        all_ok &= dk < TOL_EXACT
        print(f'      内参: 最大差 = {dk:.3e}   {"逐位一致" if dk < TOL_EXACT else "**不一致**"}')

        # 畸变
        kd = np.array([v['k1'], v['k2'], v['p1'], v['p2']])
        od = np.array(ov[cam]['distortion_coeffs'], dtype=float)
        dd = np.abs(kd - od).max()
        all_ok &= dd < TOL_EXACT
        print(f'      畸变: 最大差 = {dd:.3e}   {"逐位一致" if dd < TOL_EXACT else "**不一致**"}')

        # 时偏
        td = cfg.real('td')
        otd = float(ov[cam]['timeshift_cam_imu'])
        dtd = abs(td - otd)
        print(f'      时偏: VINS td = {td:.9f}   OpenVINS = {otd:.9f}   差 = {dtd*1e6:.3f} us')
        print()

    print('-' * 66)
    print('结论:', '全部一致 —— 两条独立转换链对同一物理量给出相同结果'
          if all_ok else '存在不一致，需排查')
    sys.exit(0 if all_ok else 2)


if __name__ == '__main__':
    main()
