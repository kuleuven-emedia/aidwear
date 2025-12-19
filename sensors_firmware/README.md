# Flash firmware on Nicla Sense ME

## Windows
1. Update the `-D` flags in the `build_fw.bat` file to include desired modalities into packets sent by the sensor.
2. Run `build_fw.bat` to compile the project with [arduino-cli](https://docs.arduino.cc/arduino-cli/).
3. Dump contents of the `nicla\build\arduino.mbed_nicla.nicla_sense` into a `nicla\fw.h` C array file with [ImHex](https://imhex.werwolv.net/) under `File > Export > Text Formatted Bytes > C Array`.
4. Update contents of the `nicla\fw.h` to end with an array length variable like `unsigned int data_len = <copy_here_array_length>;` and starting with `const unsigned char data[] = {`.
5. Copy the `nicla\fw.h` into the `failsafe_flasher\`.
6. Program the sensor board out of Arduino IDE with the `failsafe_flasher` Sketch.

### (Alternative 3-5)
Alternatively, out of Git Bash:
1. Dump binary data `echo const > nicla/fw.h && xxd -i -n data nicla/build/arduino.mbed_nicla.nicla_sense/nicla.ino.bin >> nicla/fw.h && cp nicla/fw.h failsafe_flasher/fw.h`.
2. Program the sensor board out of Arduino IDE with the `failsafe_flasher` Sketch.

## Linux
1. Update the `-D` flags in the `build_fw.sh` file to include desired modalities into packets sent by the sensor.
2. Run `. build_fw.sh` to compile the project and dump the C array into the expected format.
3. Program the sensor board out of Arduino IDE with the `failsafe_flasher` Sketch.
