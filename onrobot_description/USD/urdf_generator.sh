#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ONROBOT_DESCRIPTION_DIR="$(realpath "$SCRIPT_DIR/..")"

ros2 run xacro xacro \
  "$SCRIPT_DIR/../urdf/onrobot_rg_USD_compatible_description.urdf.xacro" \
  onrobot_description_filepath:="$ONROBOT_DESCRIPTION_DIR" \
  -o "$SCRIPT_DIR/onrobot_rg.urdf"
