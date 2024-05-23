## Generating Preferences

Application preferences are not hard-coded in the application. They are
stored in NVS and are not committed to the repository as they contain
sensitive data. The preference values are read from files in the `prefs`
directory. These will need to be manually created before generating the
NVS partition file. Each preference value is read from a unique file
which should be obvious by looking at the `preferences.csv` file.

The NVS image file is not automatically updated during the build. When
prefs values change, manually generate/flash the partition via:

```bash
./generate-nvs.sh
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

# Dumping NVS flash settings values

To view the settings values stored in NVS on a connected device:
```bash
./dump-nvs.sh
```

To write them back to preferences files for a subsequent flash via
`generate-nvs.sh`:
```bash
./dump-nvs.sh | ./nvs_to_prefs_files.py
```
