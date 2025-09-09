# Advanced Navigation Certus Mini D Notes

## Interface
The Certus is interfaced using a [compiled C code](../cpp/certus/stream_packets.c) which passes line deliminted Json data via Pipe to a [Python monitor script](../certus/run.py). 

## Setup
Setup should be done via the graphical interface of the [Advanced Navigation Mini Manager](https://www.advancednavigation.com/products/gnss-imu/minimanager-software/) application.
The application is downloaded as part of the install and can be run via
``` bash
java -jar .certus_manager.jar
```

Recommended settings are:
- Filter model: Stationary (it seems this solves AHRS without movement), unless station is truly moving (i.e. airborne)
- Output frequency: 20 Hz minimum for message 20 (system state). Message 28 is not used in this repo but recommended to leave enabled.
- Further setting recommendations TBC after testing.

## Data logging
The Python script logs the data, as well as broadcasts over ZMQ to the chosen protocol/port.
There is an option in the config files to also log more detailed binary data which can be processed by the Advanced Navigation Mini Manager application. 
However, I am unsure how much data and bandwidth this uses, so it should be tested alongside other DAQ systems before use.

## Broadcasting
The Python script broadcasts the data as Json over ZMQ as a PUB socket.
The default IPC address is /tmp/certus.sock, but this can be changed in the config file.
