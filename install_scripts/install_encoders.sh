#!/bin/bash

CWD="$(pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Install dependencies
# # None

mkdir -p ~/obs-config
cp "$SCRIPT_DIR"/../config/default_encoders_config.yaml ~/obs-config/encoders_config.yaml

echo "Encoder installation complete."



