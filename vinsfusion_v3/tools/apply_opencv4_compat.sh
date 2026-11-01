#!/usr/bin/env bash
# 让官方 VINS-Fusion 能在 OpenCV 4 上编译（Ubuntu 20.04 / 22.04，即 ROS Noetic 或 ROS2 环境）。
#
# 背景：官方 VINS-Fusion 只支持 ROS Kinetic/Melodic（Ubuntu 16.04/18.04，OpenCV 3）。
# OpenCV 4 把一批 C 风格常量从默认头里挪走了（有的移进 legacy 头，有的直接删），
# 于是 camera_models / vins_estimator / loop_fusion 编译报 "was not declared in this scope"。
# 这些常量绝大多数在 OpenCV 4 的兼容头里仍然存在，只是不再被 opencv2/opencv.hpp 自动带上。
#
# 本脚本只做一件事：在受影响的源文件顶部补一组 include。
# 不改任何一行算法逻辑、不改 CMake、不改变量语义 —— 编译产物与官方原版行为一致。
#
# 覆盖的常量（均由下列头提供）：
#   core/types_c.h              -> CV_TERMCRIT 系列
#   imgproc/types_c.h           -> CV_THRESH / CV_RETR / CV_CHAIN_APPROX / CV_SHAPE /
#                                  CV_ADAPTIVE_THRESH / CV_BGR2GRAY / CV_GRAY2BGR / CV_GRAY2RGB
#   imgproc/imgproc_c.h         -> CV_AA、CV_FONT_HERSHEY 系列
#   calib3d/calib3d_c.h         -> CV_CALIB_CB 系列
#   imgcodecs/legacy/constants_c.h -> CV_LOAD_IMAGE 系列
#
# 幂等：重复执行不会重复插入（靠 VINS_OPENCV4_COMPAT 标记判断）。
# 用法：在仓库根目录执行  bash tools/apply_opencv4_compat.sh
set -euo pipefail

MARK='VINS_OPENCV4_COMPAT'

# 注：兼容头要在任何 opencv 头之前生效，因此插到文件最顶部。
# 注释里刻意不写裸的 "CV_XXX_" 形态的标识符，否则会污染全库 grep 扫描。
COMPAT_BLOCK='// VINS_OPENCV4_COMPAT: OpenCV 4 兼容头。由 tools/apply_opencv4_compat.sh 插入。
// 这些 C 风格常量在 OpenCV 4 里已不再由 opencv2/opencv.hpp 自动带出，需显式包含。
#include <opencv2/core/types_c.h>                 // 终止条件常量
#include <opencv2/imgproc/types_c.h>              // 阈值/连通域/颜色转换/形态学常量
#include <opencv2/imgproc/imgproc_c.h>            // 抗锯齿线型、字体常量
#include <opencv2/calib3d/calib3d_c.h>            // 棋盘格检测标志
#include <opencv2/imgcodecs/legacy/constants_c.h> // 图像读取模式常量'

FILES=(
  camera_models/src/chessboard/Chessboard.cc
  camera_models/src/calib/CameraCalibration.cc
  camera_models/src/intrinsic_calib.cc
  vins_estimator/src/featureTracker/feature_tracker.cpp
  vins_estimator/src/KITTIOdomTest.cpp
  vins_estimator/src/KITTIGPSTest.cpp
  loop_fusion/src/keyframe.cpp
  loop_fusion/src/pose_graph.cpp
  loop_fusion/src/ThirdParty/DVision/BRIEF.cpp
)

changed=0
for f in "${FILES[@]}"; do
  if [[ ! -f $f ]]; then
    echo "跳过（文件不存在）: $f"
    continue
  fi
  if grep -q "$MARK" "$f"; then
    echo "已打过补丁: $f"
    continue
  fi
  tmp=$(mktemp)
  { printf '%s\n' "$COMPAT_BLOCK"; cat "$f"; } > "$tmp"
  mv "$tmp" "$f"
  echo "已打补丁: $f"
  changed=$((changed + 1))
done

echo "完成，修改 $changed 个文件。"
if (( changed > 0 )); then
  echo "撤销：git checkout -- ${FILES[*]}"
fi
