# This is a guide to install Carla in Ubuntu 22.04

* First, clone the [CARLA_0.9.14.tar.gz](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.14.tar.gz) and its [additional package](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.14.tar.gz).

* Unzip and copy the AdditionalMap package in *Import* folder, and execute ImportAssests.sh.

* For PythonAPI to connect to Carla simulator, you need Python3.7. For this install that version of python:
```
sudo apt install python3.7 python3-venv python3.7-venv
```
* Then, create and activate a virtual environment for the carla:
```
python3.7 -m venv carla-venv
source carla-venv/bin/activate
```
* After this, all the python modules you import should be in the virtual environment:
```
pip3.7 install carla
pip3.7 install numpy
pip3.7 install pygame
```
OR copy the requirements.txt file in Carla root folder and:
```
pip3.7 install -r requirements.txt
```
* To test this, in terminal:
```
./CarlaUE4.sh
```
* In another terminal:
```
cd PythonAPI/examples
python3.7 manual_control.py
```

## Reference:

* https://github.com/carla-simulator/carla/blob/master/Docs/download.md
