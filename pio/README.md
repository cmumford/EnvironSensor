## Generating Preferences

Application preferences are not committed to source, and are stored in NVS.
These are generated using `generate-nvs.py` which produces a `preferences.bin`
image file. The preference files are stored in the `prefs` directory, whihch
is not committed to the repository as it contains sensitive data. This will
need to be manually created before generating the NVS partition file.

The image file is not automatically updated. When prefs values change
manually generate the file via:

```bash
python3 generate-nvs.py
```

And then flash it with a terminal that has sourced the esp-idf export.sh file using:

```sh
parttool.py -p /dev/cu.usbserial-0001 write_partition --partition-name=nvs --input=preferences.bin
```

**Note**: You may need to terminate VSCode (or at least the serial port monitor).

More info on this process at
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_partition_gen.html

## Erasing Flash

Flash may be erased via the PlatformIO UI, or with the ESP-IDF tool:

```bash
esptool.py --port /dev/cu.usbserial-0001 erase_flash
```

When this is done the NVS partition is erased, which will require reflashing to
store the application preferences (see above).