#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
/tmp/scan3dlive_install_files/build/Livox-SDK2/samples/livox_lidar_quick_start/livox_lidar_quick_start $SCRIPT_DIR/check_lidar_config/hap_config.json