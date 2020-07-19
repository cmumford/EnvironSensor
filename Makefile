# Copyright 2019 Christopher Mumford
# This code is licensed under MIT license (see LICENSE for details)

CLANG_FORMAT=clang-format
ARDUINO=/Applications/Arduino.app/Contents/MacOS/Arduino

# Arduino default treats warnings as errors, but there is a
# bug in TinyXML which causes command-line builds to fail.
# Removing -Wextra (which means -Werror) to allow build to
# Succeed.
ARDUINO_FLAGS=--pref compiler.warning_flags.all='-Wall'

.PHONY: default
default: verify

.PHONY: format
format:
	${CLANG_FORMAT} -i \
		Arduino/EnvironSensor.ino \
		mgos/fs/main.cpp \

.PHONY: verify
verify:
	${ARDUINO} --verify ${ARDUINO_FLAGS} Arduino/EnvironSensor.ino
