import sys
import argparse
from pathlib import Path
from .tui import HealthApp


# -------------- entrypoint --------------
def main():

    p = argparse.ArgumentParser(description="Health Monitoring TUI Application")
    p.add_argument(
        "--config",
        type=Path,
        required=False,
        default=Path(Path.home(), "obs-config/tui_default_config.yaml"),
        help="Path to the configuration file",
    )
    args = p.parse_args()

    _ = sys.stdout.write(f"\33]0;{'Obs TUI Monitor'}\a")
    _ = sys.stdout.flush()

    HealthApp(args.config).run()


if __name__ == "__main__":
    main()
