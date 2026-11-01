#!/usr/bin/env python3
"""Kalibr 标定结果 -> VINS-Fusion 配置。

输入：
  camchain-imucam-*.yaml   Kalibr 相机-IMU 联合标定输出（含内参/畸变/外参/时偏）
  imu-*.yaml               Kalibr IMU 标定输出（含噪声密度与随机游走）

输出（写入 out_dir）：
  cam0.yaml / cam1.yaml                    相机内参（VINS 格式）
  <name>_stereo_imu_config.yaml            主配置

方向约定（已对照 vins_estimator/src/estimator/parameters.cpp 与官方 D435i 模板核实）：
  Kalibr 的 T_cam_imu 满足  p_cam = T_cam_imu * p_imu
  VINS-Fusion 要的 body_T_cam0 经 cv2eigen 后直接取 RIC/TIC，源码中不做任何求逆，
  故  body_T_cam0 = T_body_cam0 = inv(T_cam_imu)  —— 即 T_imu_cam。
  （官方 D435i 模板的 body_T_cam0 旋转块按 R_body_cam 解读恰好得到 body=x前/y左/z上，
   按反向解读则 body 的 x 轴朝上，故该结论可独立复核。）

td 约定：VINS 注释为 "readed image clock + td = real image clock (IMU clock)"，
  与 Kalibr 的 timeshift_cam_imu 同形式，原样搬运。

用法：
  kalibr2vins.py <camchain-imucam-*.yaml> <imu-*.yaml> <out_dir> [选项]

选项：
  --name NAME           主配置文件名前缀（默认 d430_px4）
  --imu-topic T         默认 /imu
  --cam0-topic T        默认 /camera/camera/infra1/image_rect_raw
  --cam1-topic T        默认 /camera/camera/infra2/image_rect_raw
  --output-path P       VINS 轨迹输出目录（默认 /root/calib_data/vins_output）
  --estimate-td 0|1     是否在线估计时偏（默认 0，用 Kalibr 的实测值）
  --estimate-extrinsic 0|1  是否优化外参（默认 0，信任 Kalibr）
"""
import argparse
import os
import sys

import numpy as np
import yaml


def load(p):
    """读 OpenCV FileStorage 风格的 yaml（%YAML:1.0 指令 PyYAML 不认，先剔掉）。"""
    with open(p) as f:
        lines = [ln for ln in f if not ln.startswith('%YAML')]
    return yaml.safe_load(''.join(lines))


def fmt(v):
    """足够精度且不带科学计数法尾巴的浮点表示。"""
    return repr(float(v))


def opencv_matrix(name, M, indent='   '):
    """输出 cv::FileStorage 认的 !!opencv-matrix 块（VINS 走 FileStorage 读取，
    必须是这个标签，普通嵌套列表读不出来）。"""
    M = np.asarray(M, dtype=float)
    rows, cols = M.shape
    vals = ', '.join(fmt(x) for x in M.reshape(-1))
    return (f"{name}: !!opencv-matrix\n"
            f"{indent}rows: {rows}\n"
            f"{indent}cols: {cols}\n"
            f"{indent}dt: d\n"
            f"{indent}data: [ {vals} ]\n")


def write_cam_yaml(path, K, D, w, h, cam_name='camera'):
    """VINS 相机内参文件。Kalibr 的 radtan 正好 4 参数，与 k1,k2,p1,p2 一对一。"""
    fx, fy, cx, cy = K
    k1, k2, p1, p2 = D
    with open(path, 'w') as f:
        f.write('%YAML:1.0\n---\n')
        f.write(f'model_type: PINHOLE\n')
        f.write(f'camera_name: {cam_name}\n')
        f.write(f'image_width: {w}\n')
        f.write(f'image_height: {h}\n')
        f.write('distortion_parameters:\n')
        f.write(f'   k1: {fmt(k1)}\n   k2: {fmt(k2)}\n')
        f.write(f'   p1: {fmt(p1)}\n   p2: {fmt(p2)}\n')
        f.write('projection_parameters:\n')
        f.write(f'   fx: {fmt(fx)}\n   fy: {fmt(fy)}\n')
        f.write(f'   cx: {fmt(cx)}\n   cy: {fmt(cy)}\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('camchain')
    ap.add_argument('imuyaml')
    ap.add_argument('out_dir')
    ap.add_argument('--name', default='d430_px4')
    ap.add_argument('--imu-topic', default='/imu')
    ap.add_argument('--cam0-topic', default='/camera/camera/infra1/image_rect_raw')
    ap.add_argument('--cam1-topic', default='/camera/camera/infra2/image_rect_raw')
    ap.add_argument('--output-path', default='/root/calib_data/vins_output')
    ap.add_argument('--estimate-td', type=int, default=0, choices=(0, 1))
    ap.add_argument('--estimate-extrinsic', type=int, default=0, choices=(0, 1))
    args = ap.parse_args()

    cc = load(args.camchain)
    im_raw = load(args.imuyaml)
    # Kalibr 的 imu-*.yaml 顶层是 imu0 这一层
    im = im_raw['imu0'] if 'imu0' in im_raw else im_raw
    for k in ('accelerometer_noise_density', 'gyroscope_noise_density',
              'accelerometer_random_walk', 'gyroscope_random_walk'):
        if k not in im:
            sys.exit(f'{args.imuyaml} 里缺字段 {k}')
    os.makedirs(args.out_dir, exist_ok=True)

    cams = {}
    for cam in ('cam0', 'cam1'):
        c = cc[cam]
        cams[cam] = {
            'T_cam_imu': np.array(c['T_cam_imu'], dtype=float),
            'K': [float(x) for x in c['intrinsics']],
            'D': [float(x) for x in c['distortion_coeffs']],
            'res': [int(x) for x in c['resolution']],
            'td': float(c['timeshift_cam_imu']),
            'T_cn_cnm1': np.array(c['T_cn_cnm1'], dtype=float) if 'T_cn_cnm1' in c else None,
        }

    w, h = cams['cam0']['res']
    for cam in ('cam0', 'cam1'):
        if cams[cam]['res'] != [w, h]:
            print(f"警告：{cam} 分辨率 {cams[cam]['res']} 与 cam0 {[w, h]} 不一致", file=sys.stderr)

    # 相机内参文件
    for cam in ('cam0', 'cam1'):
        write_cam_yaml(os.path.join(args.out_dir, f'{cam}.yaml'),
                       cams[cam]['K'], cams[cam]['D'], w, h)

    # 外参：VINS 要的正是 inv(T_cam_imu)
    T_body_cam = {cam: np.linalg.inv(cams[cam]['T_cam_imu']) for cam in ('cam0', 'cam1')}

    cfg = os.path.join(args.out_dir, f'{args.name}_stereo_imu_config.yaml')
    with open(cfg, 'w') as f:
        f.write('%YAML:1.0\n\n')
        f.write('# 由 tools/kalibr2vins.py 生成，请勿手改；改请改脚本后重跑。\n')
        f.write(f'# 来源: {os.path.basename(args.camchain)} + {os.path.basename(args.imuyaml)}\n')
        f.write('# body_T_cam* = inv(Kalibr T_cam_imu)；td = Kalibr timeshift_cam_imu\n\n')
        f.write('#common parameters\n')
        f.write('imu: 1\n')
        f.write('num_of_cam: 2\n\n')
        f.write(f'imu_topic: "{args.imu_topic}"\n')
        f.write(f'image0_topic: "{args.cam0_topic}"\n')
        f.write(f'image1_topic: "{args.cam1_topic}"\n')
        f.write(f'output_path: "{args.output_path}"\n\n')
        f.write('cam0_calib: "cam0.yaml"\n')
        f.write('cam1_calib: "cam1.yaml"\n')
        f.write(f'image_width: {w}\n')
        f.write(f'image_height: {h}\n\n')
        f.write('# Extrinsic parameter between IMU and Camera.\n')
        f.write(f'estimate_extrinsic: {args.estimate_extrinsic}\n')
        f.write('# 0 = 信任并固定标定值；1 = 以标定值为初值继续优化\n\n')
        f.write(opencv_matrix('body_T_cam0', T_body_cam['cam0']))
        f.write('\n')
        f.write(opencv_matrix('body_T_cam1', T_body_cam['cam1']))
        f.write('\n')
        f.write('#Multiple thread support\n')
        f.write('multiple_thread: 1\n\n')
        f.write('#feature traker paprameters\n')
        f.write('max_cnt: 150\n')
        f.write('min_dist: 30\n')
        f.write('freq: 10\n')
        f.write('F_threshold: 1.0\n')
        f.write('show_track: 0\n')
        f.write('flow_back: 1\n\n')
        f.write('#optimization parameters\n')
        f.write('max_solver_time: 0.04\n')
        f.write('max_num_iterations: 8\n')
        f.write('keyframe_parallax: 10.0\n\n')
        f.write('#imu parameters\n')
        f.write('# 来源：Kalibr imu-*.yaml。注意：这四个值 Kalibr 未做修改，\n')
        f.write('# 实际是 PX4 的典型初值（静止段不足，随机游走项为 NaN），非本机实测。\n')
        f.write(f"acc_n: {fmt(im['accelerometer_noise_density'])}\n")
        f.write(f"gyr_n: {fmt(im['gyroscope_noise_density'])}\n")
        f.write(f"acc_w: {fmt(im['accelerometer_random_walk'])}\n")
        f.write(f"gyr_w: {fmt(im['gyroscope_random_walk'])}\n")
        f.write('g_norm: 9.805\n\n')
        f.write('#unsynchronization parameters\n')
        f.write(f'estimate_td: {args.estimate_td}\n')
        f.write(f"td: {fmt(cams['cam0']['td'])}   # Kalibr timeshift_cam_imu (cam0)\n\n")
        f.write('#loop closure parameters\n')
        f.write('load_previous_pose_graph: 0\n')
        f.write(f'pose_graph_save_path: "{args.output_path}/pose_graph/"\n')
        f.write('save_image: 0\n')

    # ---- 摘要与自检 ----
    print(f'输出目录: {args.out_dir}')
    print(f'  主配置: {os.path.basename(cfg)}')
    print(f'  相机内参: cam0.yaml  cam1.yaml')
    print()
    for cam in ('cam0', 'cam1'):
        K = cams[cam]['K']
        t = T_body_cam[cam][:3, 3]
        print(f"{cam}: fx={K[0]:.3f} fy={K[1]:.3f} cx={K[2]:.3f} cy={K[3]:.3f}  "
              f"|t_body_cam|={np.linalg.norm(t)*1000:.2f} mm  td={cams[cam]['td']:.9f}")
    print()
    if cams['cam1']['T_cn_cnm1'] is not None:
        b = cams['cam1']['T_cn_cnm1'][:3, 3]
        print(f"双目基线 (Kalibr T_cn_cnm1): {np.linalg.norm(b)*1000:.4f} mm")
    print(f"两相机 td 之差: {abs(cams['cam0']['td'] - cams['cam1']['td'])*1e6:.3f} us")
    print()
    print('外参正交性自检 (R^T R 应为单位阵, det 应为 1):')
    for cam in ('cam0', 'cam1'):
        R = T_body_cam[cam][:3, :3]
        err = np.abs(R.T @ R - np.eye(3)).max()
        print(f'  {cam}: max|R^T R - I| = {err:.3e}   det = {np.linalg.det(R):.12f}')

    os.makedirs(args.output_path, exist_ok=True)


if __name__ == '__main__':
    main()
