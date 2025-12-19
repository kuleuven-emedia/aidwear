#!/bin/bash
arduino-cli compile -b arduino:mbed_nicla:nicla_sense -e --build-property "build.extra_flags=-DIS_GYR -DIS_EULER" nicla
echo const > nicla/fw.h && xxd -i -n data nicla/build/arduino.mbed_nicla.nicla_sense/nicla.ino.bin >> nicla/fw.h
cp nicla/fw.h failsafe_flasher/fw.h
