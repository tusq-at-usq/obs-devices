# UniSQ Imaging and Diagnostics Device (IDD) Software

Software to receive, process, broadcast, log, and monitor data from external (non camera) devices, including
- Time server sources
- Encoders
- IMUs

Emphasis is placed on:
- Reliability and robustness
- Simplicity and human oversight
- Low latency and event-driven (where possible)

## Features
- Installation and setup of environment, dependencies and permissions
- Receive data from devices via systemctl daemon services and broadcast to ZMQ PUB sockets
- Monitor the status of devices via a TUI dashboard
- Log data to CSV files

## Devices
- [Local network Time Server](docs/ntp.md)
- [Encoder board](docs/encoder.md)
- [Advanced Navigation Certus Mini D IMU](docs/imu.md)


