
from pathlib import Path
import sys
import os
from .run import CertusBroadcaster

def main():

    _ = sys.stdout.write(f"\33]0;{'IMU logging and streaming'}\a")
    _ = sys.stdout.flush()

    default_config_path = os.path.join(Path.home(), "obs-config/certus_imu_config.yaml")
    broadcaster = CertusBroadcaster(config_path=default_config_path)


    broadcaster.run()


if __name__ == "__main__":
    main()
