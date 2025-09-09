import os
import glob
import yaml
import numpy as np
import zmq
import subprocess
import time
import select
import json

from ..discovery import port_by_vidpid

CERTUS_EXEC = os.path.join(
    os.path.dirname(__file__), "../", "cpp", "certus", "stream_packets"
)


class CertusBroadcaster:
    def __init__(self, config_path: str) -> None:
        """Class to handle logging and broadcasting Certus IMU data."""

        self.proc = None
        self.config = None
        self.context = None
        self.socket = None
        self.log_idx = 0
        self.log_row_idx = 0

        # Read config
        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)

        # Open socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.set_hwm(1)
        self.socket.setsockopt(zmq.LINGER, 0)
        if self.config["protocol"] == "IPC":
            self.socket.bind(f"ipc://{self.config['address']}")
        elif self.config["protocol"] == "TCP":
            self.socket.bind(f"tcp://*:{self.config['address']}")
        else:
            raise ValueError("Unsupported protocol in config")

        # Create initial log file
        self.create_log_file(self.log_idx)

        # Start certus executable
        self.reconnect_loop()

    def read_config(self, config_path: str) -> None:
        """Read configuration from YAML file."""

        default = {
            "log_raw": False,
            "log_dir": "./idd_logs",
            "log_max_rows": 100000,
            "vidpid": "0403:6001",
            "baudrate": 115200,
            "protocol": "IPC",
            "address": "/tmp/imu.sock",
        }

        # Add missing entries with default
        with open(config_path, "r") as file:
            self.config = {**default, **yaml.safe_load(file)}

    def reconnect_loop(self, timeout=1) -> None:
        """Attempt to reconnect every `timeout` seconds."""
        while True:
            if self.reconnect():
                print("Reconnected to Certus IMU.")
                return
            print(f"Reconnection failed. Retrying in {timeout} seconds...")
            time.sleep(timeout)

    def reconnect(self) -> bool:
        """Reconnect to the Certus IMU."""
        try:
            if self.proc is not None:
                if self.proc.poll() is None:
                    self.proc.terminate()
                    self.proc.wait()

            ser_port = port_by_vidpid(self.config["vidpid"])
            if ser_port is None:
                raise RuntimeError("Could not find Certus IMU. Is it connected?")

            self.proc = subprocess.Popen(
                [
                    CERTUS_EXEC,
                    ser_port,
                    str(self.config["baudrate"]),
                    str(int(self.config["log_raw"])),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            return True
        except Exception as e:
            print(f"Reconnection failed: {e}")
            return False

    def create_log_file(self, idx: int = 0) -> None:
        """Create a new log file with suffix idx."""

        time_str = time.strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.config["log_dir"], f"{time_str}_{idx}.csv")
        os.makedirs(self.config["log_dir"], exist_ok=True)
        with open(self.log_file, "w") as f:
            f.write("timestamp,Lat,Lon,Alt,Roll,Pitch,Heading\n")
        print(f"Logging to {self.log_file}")

    def append_to_log(self, data: dict) -> None:
        """Append a line to the log file."""

        with open(self.log_file, "a") as f:
            f.write(
                f"{data['Sec']},{data['Lat']},{data['Lon']},{
                    data['Alt']},{data['Roll']},{data['Pitch']},{data['Head']}\n"
            )
        self.log_row_idx += 1
        if self.log_row_idx >= self.config["log_max_rows"]:
            self.log_idx += 1
            self.log_row_idx = 0
            self.create_log_file(self.log_idx)

    def broadcast(self, data: dict) -> None:
        """Broadcast data over ZMQ socket."""

        self.socket.send_json(data)

    def run(self) -> None:
        """Main loop to read and broadcast data."""

        if self.proc is None or self.proc.stdout is None:
            self.reconnect_loop()

        while True:
            ready, _, _ = select.select([self.proc.stdout], [], [], 0.5)
            if ready:
                line = self.proc.stdout.readline()
                try:
                    data = json.loads(line)
                    self.append_to_log(data)
                    self.broadcast(data)
                except:
                    pass
            else:
                self.reconnect_loop()


if __name__ == "__main__":
    default_config_path = os.path.join(os.path.dirname(__file__), "certus_config.yaml")
    broadcaster = CertusBroadcaster(config_path=default_config_path)
    broadcaster.run()
