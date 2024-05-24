#!/usr/bin/env python3

import fileinput
import re
import sys

"""A utility script to read the output produced by dump-nvs.sh and write the
NVS values back to the *.txt files in /prefs."""
var_mappings = {
    "mqtt-password": "prefs/mqtt-password.txt",
    "mqtt-uri": "prefs/mqtt-uri.txt",
    "mqtt-username": "prefs/mqtt-username.txt",
    "sensor-location": "prefs/sensor-location.txt",
    "sensor-name": "prefs/sensor-name.txt",
    "sensor-type": "prefs/sensor-type.txt",
    "serial-number": "prefs/serial-number.txt",
    "sleep-duration": "prefs/sleep-duration.txt",
    "wifi-password": "prefs/wifi-password.txt",
    "wifi-ssid": "prefs/wifi-ssid.txt",
}
keys = set(var_mappings.keys())
reg = re.compile(r"^prefs:(.+) = b'([^\\]+).+$", re.IGNORECASE)
for line in fileinput.input():
    m = reg.match(line.strip())
    if m:
        key = m.group(1)
        val = m.group(2)
        if key not in var_mappings:
            print(f'No mapping for key "{key}"', file=sys.stderr)
            sys.exit(1)
        keys.remove(key)
        with open(var_mappings[key], 'w') as f:
            f.write(val)

if len(keys) > 0:
    print(f"nvs had missing values for {keys}", file=sys.stderr)
    print("Manually set these values in the approprite files.")
    for k in keys:
        print(f"  key: {k} in {var_mappings[k]}")
