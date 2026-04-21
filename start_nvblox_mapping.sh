#!/bin/bash
# ============================================================================
# Nvblox 建圖腳本 — cuVSLAM + nvblox 純視覺建圖（無需 LiDAR）
#
# 使用方式：
#   1. 先啟動 RealSense 和 wl_base_node
#   2. 執行本腳本：bash start_nvblox_mapping.sh
#   3. 遙控機器人遍歷環境
#   4. 保存地圖：ros2 service call /nvblox/save_map std_srvs/srv/Trigger
#
# 可選參數：
#   --voxel-size 0.05       體素大小（米）
#   --max-dist 5.0          最大整合距離（米）
#   --slice-height 0.1      切片高度（米）
#   --save-name my_map      地圖文件名
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Defaults
VOXEL_SIZE="0.05"
MAX_DIST="5.0"
SLICE_HEIGHT="0.1"
SAVE_NAME="nvblox_map"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --voxel-size) VOXEL_SIZE="$2"; shift ;;
        --max-dist) MAX_DIST="$2"; shift ;;
        --slice-height) SLICE_HEIGHT="$2"; shift ;;
        --save-name) SAVE_NAME="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

echo "============================================"
echo "  Nvblox Mapping (cuVSLAM + nvblox)"
echo "  Voxel size:  ${VOXEL_SIZE}m"
echo "  Max dist:    ${MAX_DIST}m"
echo "  Slice height: ${SLICE_HEIGHT}m"
echo "  Save name:   ${SAVE_NAME}"
echo "============================================"

source /opt/ros/humble/setup.bash 2>/dev/null

ros2 launch "${SCRIPT_DIR}/nvblox_mapping.launch.py" \
    voxel_size:="${VOXEL_SIZE}" \
    max_integration_distance:="${MAX_DIST}" \
    slice_height:="${SLICE_HEIGHT}" \
    save_name:="${SAVE_NAME}"
