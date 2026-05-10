#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ONROBOT_DESCRIPTION_DIR="$(realpath "$SCRIPT_DIR/..")"

# First argument = gripper model
# Default = rg2_v1
GRIPPER_MODEL="${1:-rg2_v1}"

ros2 run xacro xacro \
  "$SCRIPT_DIR/../urdf/onrobot_rg_USD_compatible_description.urdf.xacro" \
  onrobot_description_filepath:="$ONROBOT_DESCRIPTION_DIR" \
  gripper_model:="$GRIPPER_MODEL" \
  -o "$SCRIPT_DIR/onrobot_${GRIPPER_MODEL}.urdf"

echo "Generated: $SCRIPT_DIR/onrobot_${GRIPPER_MODEL}.urdf"
