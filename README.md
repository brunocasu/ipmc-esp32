# Inteligent Platform Management Controller for the ESP32 microcontroller

This implementation serves as an example project for the **OpenIPMC** software, as a demontration of the software flexibility and eficiency.
The repository contains the IOs interface and the I2C peripheral adapted to the ESP32 in order to run the IPMController.

**NOTE:** The I2C driver, provided by the IDF platform, was not compatible with the IPMBus application required to run the IPMC. For this reason
the driver had to be modified, and by this, a fork of the esp-idf project was made. Is recommended to clone the example project with the forked IDF
repository as a submodule instead of using the main repository maintained by the developers, while a merge request is still beeing organized.

## How to install

Choose a folder to clone the repository, then run: 
```console
git clone --recurse-submodules -j8 git@github.com:brunocasu/ipmc-esp32.git
```
This will download the repository with all the submodules in it.

To install the IDF platform, go into the ```/esp-idf``` folder and run ```install.sh```
```console
cd ~/user_path/ipmc-esp32/esp-idf
./install.sh
```
After the IDF installation you will need to create the path to compile the project
Go to the ```idf_project``` folder and run ```export.sh``` from that folder
```console
cd ~/user_path/ipmc-esp32/idf-project
. $HOME/user_path/ipmc-esp32/esp-idf/export.sh
```

Now the project can be compiled
```console
idf.py build
```
If some configuration in the project is desired run ```console idf.py menuconfig```

## Hardware setup


