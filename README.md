# Inteligent Platform Management Controller for the ESP32 microcontroller

This implementation serves as an demonstration project for the **OpenIPMC** software, as a proof of the software flexibility and eficiency.
The repository contains the IOs interface and the I2C peripheral adapted to the ESP32 in order to run the IPMController.

**NOTE:** The I2C driver, provided by the IDF platform, was not compatible with the IPMBus application required to run the IPMC. For this reason,
the driver had to be modified, and by this, a fork of the esp-idf project was made. Is recommended to clone the example project with the forked IDF
repository as a submodule, instead of using the main repository maintained by the developers, while a merge request is still beeing organized.
Besides that, the FreeRTOS provided in the esp-idf has a different folder path than the one used in the #includes of the OpenIPMC code. Therefore, a branch was created in 
a forked repository called esp32_custom, which adapted the code to correct the FreeRTOS path and use the "printf" function available in the IDF driver.

## How to install

Using your console, choose and switch to a folder to clone the repository (as a sugestion, create a folder named "clones" in your home directory). 
```shell
cd ~
mkdir clones
cd clones
```
To clone the project use the following script:
```shell
git clone --recurse-submodules -j8 git@github.com:brunocasu/ipmc-esp32.git
```
This will download the repository with all the submodules in it.

**NOTE:** This operation may take some time as the esp-idf repository has a large number of files and submodules.

To install the IDF platform, go into the ```/esp-idf``` folder and execute ```install.sh```
```shell
cd ~/clones/ipmc-esp32/esp-idf
./install.sh
```

After the IDF installation you will need to create the path to compile the project.
Go to the ```idf_project``` folder and execute ```export.sh``` from that folder:
```shell
cd ~/clones/ipmc-esp32/idf-project
. $HOME/clones/ipmc-esp32/esp-idf/export.sh
```

Now the project is ready to be compiled. But before that is highly recommended to switch branches in the openipmc submodule, for consistency purposes.
Go into the ```/openipmc``` folder and run a git checkout command, as it follows:
```shell
cd ~/clones/ipmc-esp32/idf-project/main/openipmc
git checkout esp32_custom
```
You can also run a ```git status``` command to check if you are using the correct branch of the repository.


To compile the demonstration project, run the following command from the folder you created the path (```/idf-project```):
```shell
idf.py build
```
To program the board and monitor the USB port use:
```shell 
idf.py -p (USR_PORT) flash
idf.py -p (USR_PORT) monitor
```
If is necessary to change any configuration of the IDF project run:
```shell
idf.ṕy menuconfig
```
## Hardware setup


