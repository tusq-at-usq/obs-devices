# Remote imaging tools

This repo contains three main components:

1. Setup scripts for fresh Linux installs (Ubuntu 24.04 tested)
2. Common utility functions for data acquisition
3. A TUI dashboard for monitoring device status

![TUI dashboard example](docs/tui_example.png)

## PC Setup

This repository is designed to be cloned onto a fresh Linux install (Ubuntu 24.04 tested).
It contains a bash script `pc_setup/pc_setup.sh` which will setup dependencies, settings, permissions, an `(obs)` python environment and useful applications. 

## Common utility scripts

The `obs_utils` package contains common utility functions for device monitoring scripts.
It is installed as part of the PC setup script, but can also be installed separately by running
``` bash
uv pip install -e .
```

## TUI status monitor

## Related repositories

This repository is designed to work alongside other obs repositories:
- [obs_certus](https://github.com/tusq-at-usq/obs-certus): Advanced Navigation Certus Mini D GNSS receiver interface
- [obs_encoder]
- more TBC

## Networking
This and related repositories makes use of ZMQ for inter-process communication, for various reasons (including allowing true multi-threading).
Device monitor scripts broadcast data as PUB sockets, and other users can extract this data as SUB sockets.
When this is only done on a single local machine the IPC protocol is recommended.
When multiple machines are used, TCP is required.
The protocol and address can be specified in the YAML config files for each device.

## Device identification
The most reliable way to identify a USB serial device is by the vendor and product ID (VID:PID).
This an be queried by `$ lsusb` (although you will have to identify the device).
Alternatively, query `$ dmesg | grep -1 usb` after plugging it in to see recent USB activity.
The VID:PID is constant for device class, regardless of which individual device, PC, or port is used.
This is much more reliable than specifying port directly, which are assigned dynamically.
The VID:PID should be specified in the YAML file for each device - defaults are included for known devices.
