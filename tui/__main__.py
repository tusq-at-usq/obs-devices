import argparse
from pathlib import Path
from .tui import HealthApp

# -------------- entrypoint --------------
def main():
    filepath = Path(__file__).parent.resolve()
    p = argparse.ArgumentParser(description="Health Monitoring TUI Application")
    p.add_argument("--config", type=Path, required=False, default=Path(filepath, "$HOME/obs-config/config.yaml"), help="Path to the configuration file")
    args = p.parse_args()
    HealthApp(args.config).run()


if __name__ == "__main__":
    main()
