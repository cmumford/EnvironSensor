#!/usr/bin/env python3

import subprocess
import os


def esp_idf_path():
    return os.path.expanduser('~/esp/v5.1.2/esp-idf')


def nvs_partition_gen_path():
    return os.path.join(esp_idf_path(), 'components', 'nvs_flash',
                        'nvs_partition_generator', 'nvs_partition_gen.py')


def esptool_path():
    return os.path.join(esp_idf_path(), 'components', 'esptool_py',
                        'esptool', 'esptool.py')


def create_image():
    # Docs says output size must me a multiple of 4096 bytes. App says minimum 0x3000.
    output_size = '0x6000'
    cmd = ['python3', nvs_partition_gen_path(), 'generate', 'preferences.csv',
           'preferences.bin', output_size]
    subprocess.check_call(cmd)


def main():
    create_image()
    # upload_nvs = esptool.py write_flash 0x9000 preferences.bin


if __name__ == '__main__':
    main()
