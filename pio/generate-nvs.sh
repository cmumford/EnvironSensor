#!/bin/bash -e

if [[ -z "${ESP_IDF_VERSION}" ]]; then
  echo "ESP-IDF not loaded."
  echo "Probably need to do something like:"
  echo "source ~/esp/v5.1.2/esp-idf/export.sh"
  exit 1
fi

PARTITION_SIZE="0x6000"
DEVICE="/dev/cu.usbserial-0001"

NVS_PARTITION_GEN=${IDF_PATH}/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py

${NVS_PARTITION_GEN} generate preferences.csv preferences.bin ${PARTITION_SIZE}

parttool.py -p $DEVICE write_partition --partition-name=nvs --input=preferences.bin

echo "NVS partition build/flash complete."
