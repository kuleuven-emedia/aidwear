@echo on
arduino-cli compile -b arduino:mbed_nicla:nicla_sense -e --build-property "build.extra_flags=-DIS_GYR -DIS_EULER" nicla
