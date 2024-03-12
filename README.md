# gdolib

Garage door opener library for controlling security+ version 1.0 and 2.0 garage door openers for ESP32 microcontrollers.


## Prerequisities:
* ESP-IDF v4.4+

## Building:
* Setup esp-idf, see instructions [here](https://docs.espressif.com/projects/esp-idf/en/v5.2.1/esp32/get-started/index.html).
* Follow the directions [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-py.html) to create a new project and copy the files from the examples folder in this repo into the new project folder.
* cd into the folder, run <path-to-esp-idf>/export.sh if necessary to setup the environment.
* run `idf.py  build`

At this point it should have successfully built the project and the output will be in the build folder inside the project folder.
To extract the library, navigate to `build/esp-idf/gdolib` and copy the `libgdolib.a` file to another project folder where you want it to be statically linked.


