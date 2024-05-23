#!/bin/bash

if [[ -z "${ESP_IDF_VERSION}" ]]; then
  echo "ESP-IDF not loaded."
  echo "Probably need to do something like:"
  echo "source ~/esp/v5.1.2/esp-idf/export.sh"
  exit 1
fi

DEVICE="/dev/cu.usbserial-0001"

parttool.py -p $DEVICE read_partition --partition-name=nvs --output=nvs.bin
rc=$?
if [ $rc -ne 0 ]; then
  echo
  echo "Error writing NVS partition."
  echo "Check the VSCode serial monitor which may be interfering with this write."
  exit $rc
fi

NVS_TOOL=${IDF_PATH}/components/nvs_flash/nvs_partition_tool/nvs_tool.py

${NVS_TOOL} --dump=minimal --color=auto --format=text nvs.bin | grep prefs:
rc=$?
if [ $rc -ne 0 ]; then
  echo "Error dumping NVS partition."
  exit $rc
fi

