#!/bin/bash
# Incremental rebuild of the spectrometer firmware (camera.c fix)
export PATH=/tmp/py311shim:$PATH
source /tmp/.espressif/v5.5.4/esp-idf/export.sh >/dev/null 2>&1
cd /home/ubuntu/workspace/spectrometer-esp32
idf.py build > /tmp/build10.log 2>&1
rc=$?
echo "BUILD_EXIT=$rc"
grep -E 'error:|undefined reference' /tmp/build10.log | head -6
grep -E 'binary size|bytes free|from 2789' /tmp/build10.log | tail -2
