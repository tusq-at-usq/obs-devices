#!/bin/bash

CWD="$(pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Install dependencies
sudo apt-get update
sudo apt-get install -y build-essential cmake libssl-dev libboost-all-dev libjansson-dev

# JRE from Adoptium
sudo apt install -y wget apt-transport-https gpg
wget -qO - https://packages.adoptium.net/artifactory/api/gpg/key/public | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/adoptium.gpg > /dev/null
echo "deb https://packages.adoptium.net/artifactory/deb $(awk -F= '/^VERSION_CODENAME/{print$2}' /etc/os-release) main" | sudo tee /etc/apt/sources.list.d/adoptium.list
sudo apt update # update if you haven't already
sudo apt install temurin-11-jre


# Download and set up Certus Mini Manager
curl -L https://www.advancednavigation.com/downloads/latest/certus-mini-d/CertusMiniManager-7.5.zip -o ~/CertusMiniManager-7.5.zip
unzip ~/CertusMiniManager-7.5.zip -d ~/CertusMiniManager
rm ~/CertusMiniManager-7.5.zip


# Compile Certus C++ SDK interface
# Store current directory
cd "$SCRIPT_DIR/../obs_certus/cpp/certus"
make
cd "$CWD"

mkdir -p ~/obs-config
cp "$SCRIPT_DIR"/../config/default_config.yaml ~/obs-config/certus_certus_config.yaml

